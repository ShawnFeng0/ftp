/// @file mavlink_ftp.cpp
///	@author px4dev, Don Gagne <don@thegagnes.com>

#include "file_server.h"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <utility>

#include "crc32.h"
#include "ulog/ulog.h"

#define FTP_DEBUG

/**
 * Get absolute time in [us] (does not wrap).
 */
static inline uint64_t absolute_time_us() {
  struct timespec ts = {};
  clock_gettime(CLOCK_MONOTONIC, &ts);

  uint64_t result = (uint64_t)(ts.tv_sec) * 1000000;
  result += ts.tv_nsec / 1000;

  return result;
}

using namespace uftp;

FileServer::FileServer(std::string root_directory, void *user_data,
                       WriteCallback write_callback)
    : root_directory_(std::move(root_directory)),
      user_data_(user_data),
      write_callback_(write_callback) {
  // initialize session
  session_info_.fd = -1;
}

FileServer::~FileServer() {
  delete[] work_buffer1_;
  delete[] work_buffer2_;
}

unsigned FileServer::get_size() const {
  return session_info_.stream_download ? kMaxPacketLength : 0;
}

/// @brief Processes an FTP message
void FileServer::ProcessRequest(Payload *payload) {
  bool stream_send = false;

  ErrorCode errorCode = kErrNone;

  auto payload_str = payload->to_string();

#ifdef FTP_DEBUG
  LOGGER_TOKEN(payload_str.c_str());
#endif

  if (!ensure_buffers_exist()) {
    LOGGER_ERROR("Failed to allocate buffers");
    errorCode = kErrFailErrno;
    errno = ENOMEM;
    goto out;
  }

  // basic sanity checks; must validate length before use
  if (payload->size > kMaxDataLength) {
    errorCode = kErrInvalidDataSize;
    goto out;
  }

  // check the sequence number: if this is a resent request, resend the last
  // response
  if (last_reply_valid_) {
    auto *last_payload = reinterpret_cast<Payload *>(last_reply_);

    if (payload->seq_number + 1 == last_payload->seq_number) {
      // this is the same request as the one we replied to last. It means the
      // (n)ack got lost, and the GCS resent the request
      if (write_callback_) {
        write_callback_(user_data_,
                        reinterpret_cast<const char *>(last_payload),
                        sizeof(Payload) + last_payload->size);
      }
      return;
    }
  }

#ifdef FTP_DEBUG
  LOGGER_INFO("ftp: opc %u size %u offset %u", payload->opcode, payload->size,
              payload->offset);
#endif

  switch (payload->opcode) {
    case kCmdNone:
      break;

    case kCmdTerminateSession:
      errorCode = WorkTerminate(payload);
      break;

    case kCmdResetSessions:
      errorCode = WorkReset(payload);
      break;

    case kCmdListDirectory:
      errorCode = WorkList(payload);
      break;

    case kCmdOpenFileRO:
      errorCode = WorkOpen(payload, O_RDONLY);
      break;

    case kCmdCreateFile:
      errorCode = WorkOpen(payload, O_CREAT | O_TRUNC | O_WRONLY);
      break;

    case kCmdOpenFileWO:
      errorCode = WorkOpen(payload, O_CREAT | O_WRONLY);
      break;

    case kCmdReadFile:
      errorCode = WorkRead(payload);
      break;

    case kCmdBurstReadFile:
      errorCode = WorkBurst(payload);
      stream_send = true;
      break;

    case kCmdWriteFile:
      errorCode = WorkWrite(payload);
      break;

    case kCmdRemoveFile:
      errorCode = WorkRemoveFile(payload);
      break;

    case kCmdRename:
      errorCode = WorkRename(payload);
      break;

    case kCmdTruncateFile:
      errorCode = WorkTruncateFile(payload);
      break;

    case kCmdCreateDirectory:
      errorCode = WorkCreateDirectory(payload);
      break;

    case kCmdRemoveDirectory:
      errorCode = WorkRemoveDirectory(payload);
      break;

    case kCmdCalcFileCRC32:
      errorCode = WorkCalcFileCrc32(payload);
      break;

    default:
      errorCode = kErrUnknownCommand;
      break;
  }

out:
  payload->seq_number++;

  // handle success vs. error
  if (errorCode == kErrNone) {
    payload->req_opcode = payload->opcode;
    payload->opcode = kRspAck;

  } else {
    int r_errno = errno;
    payload->req_opcode = payload->opcode;
    payload->opcode = kRspNak;
    payload->size = 1;

    if (r_errno == EEXIST) {
      errorCode = kErrFailFileExists;

    } else if (r_errno == ENOENT && errorCode == kErrFailErrno) {
      errorCode = kErrFileNotFound;
    }

    payload->data[0] = errorCode;

    if (errorCode == kErrFailErrno) {
      payload->size = 2;
      payload->data[1] = r_errno;
    }
  }

  last_reply_valid_ = false;

  // Stream download replies are sent through mavlink stream mechanism. Unless
  // we need to Nack.
  if (!stream_send || errorCode != kErrNone) {
    // respond to the request
    Reply(payload);
  }
}

bool FileServer::ensure_buffers_exist() {
  last_work_buffer_access_ = absolute_time_us();

  if (!work_buffer1_) {
    work_buffer1_ = new char[work_buffer1_len_];
  }

  if (!work_buffer2_) {
    work_buffer2_ = new char[work_buffer2_len_];
  }

  return work_buffer1_ && work_buffer2_;
}

/// @brief Sends the specified FTP response message out through mavlink
void FileServer::Reply(Payload *payload) {
  // keep a copy of the last sent response ((n)ack), so that if it gets lost and
  // the GCS resends the request, we can simply resend the response. we only
  // keep small responses to reduce RAM usage and avoid large memcpy's. The
  // larger responses are all data retrievals without side-effects, meaning it's
  // ok to reexecute them if a response gets lost
  if (payload->size <= sizeof(uint32_t)) {
    last_reply_valid_ = true;
    memcpy(last_reply_, payload, sizeof(Payload) + payload->size);
  }

#ifdef FTP_DEBUG
  LOGGER_INFO("FTP: %s seq_number: %d",
              payload->opcode == kRspAck ? "Ack" : "Nak", payload->seq_number);
#endif

  if (write_callback_) {
    write_callback_(user_data_, reinterpret_cast<const char *>(payload),
                    sizeof(Payload) + payload->size);
  }
}

/// @brief Responds to a List command
FileServer::ErrorCode FileServer::WorkList(Payload *payload) {
  strncpy(work_buffer1_, root_directory_.c_str(), work_buffer1_len_);
  strncpy(work_buffer1_ + root_directory_.length(), payload->data_as_c_str(),
          work_buffer1_len_ - root_directory_.length());
  // ensure termination
  work_buffer1_[work_buffer1_len_ - 1] = '\0';

  ErrorCode errorCode = kErrNone;
  unsigned offset = 0;

  DIR *dp = opendir(work_buffer1_);

  if (dp == nullptr) {
    LOGGER_WARN("File open failed %s", work_buffer1_);
    return kErrFileNotFound;
  }

#ifdef FTP_DEBUG
  LOGGER_INFO("FTP: list %s offset %d", work_buffer1_, payload->offset);
#endif

  // move to the requested offset
  auto requested_offset = payload->offset;

  while (requested_offset-- > 0 && readdir(dp)) {
  }

  for (;;) {
    errno = 0;
    struct dirent *result = readdir(dp);

    // read the directory entry
    if (result == nullptr) {
      if (errno) {
        LOGGER_WARN("readdir failed");
        payload->data[offset++] = kDirentSkip;
        *((char *)&payload->data[offset]) = '\0';
        offset++;
        payload->size = offset;
        closedir(dp);

        return errorCode;
      }

      // no more entries?
      if (payload->offset != 0 && offset == 0) {
        // User is requesting subsequent dir entries but there were none. This
        // means the user asked to seek past EOF.
        errorCode = kErrEOF;
      }

      // Otherwise we are just at the last directory entry, so we leave the
      // errorCode at kErrorNone to signal that
      break;
    }

    uint32_t fileSize = 0;
    char direntType;

    // Determine the directory entry type
    switch (result->d_type) {
#ifdef __PX4_NUTTX

      case DTYPE_FILE: {
#else

      case DT_REG: {
#endif
        // For files we get the file size as well
        direntType = kDirentFile;
        int ret = snprintf(work_buffer2_, work_buffer2_len_, "%s/%s",
                           work_buffer1_, result->d_name);
        bool buf_is_ok = ((ret > 0) && (ret < work_buffer2_len_));

        if (buf_is_ok) {
          struct stat st {};

          if (stat(work_buffer2_, &st) == 0) {
            fileSize = st.st_size;
          }
        }

        break;
      }

#ifdef __PX4_NUTTX

      case DTYPE_DIRECTORY:
#else
      case DT_DIR:
#endif
        if (strcmp(result->d_name, ".") == 0 ||
            strcmp(result->d_name, "..") == 0) {
          // Don't bother sending these back
          direntType = kDirentSkip;

        } else {
          direntType = kDirentDir;
        }

        break;

      default:
        // We only send back file and diretory entries, skip everything else
        direntType = kDirentSkip;
    }

    if (direntType == kDirentSkip) {
      // Skip send only dirent identifier
      work_buffer2_[0] = '\0';

    } else if (direntType == kDirentFile) {
      // Files send filename and file length
      int ret = snprintf(work_buffer2_, work_buffer2_len_, "%s\t%d",
                         result->d_name, fileSize);
      bool buf_is_ok = ((ret > 0) && (ret < work_buffer2_len_));

      if (!buf_is_ok) {
        work_buffer2_[work_buffer2_len_ - 1] = '\0';
      }

    } else {
      // Everything else just sends name
      strncpy(work_buffer2_, result->d_name, work_buffer2_len_);
      work_buffer2_[work_buffer2_len_ - 1] = '\0';
    }

    size_t nameLen = strlen(work_buffer2_);

    // Do we have room for the name, the one char directory identifier and the
    // null terminator?
    if ((offset + nameLen + 2) > kMaxDataLength) {
      break;
    }

    // Move the data into the buffer
    payload->data[offset++] = direntType;
    strcpy((char *)&payload->data[offset], work_buffer2_);
#ifdef FTP_DEBUG
    LOGGER_INFO("FTP: list %s %s", work_buffer1_,
                (char *)&payload->data[offset - 1]);
#endif
    offset += nameLen + 1;
  }

  closedir(dp);
  payload->size = offset;

  return errorCode;
}

/// @brief Responds to an Open command
FileServer::ErrorCode FileServer::WorkOpen(Payload *payload, int oflag) {
  if (session_info_.fd >= 0) {
    LOGGER_ERROR("FTP: Open failed - out of sessions\n");
    return kErrNoSessionsAvailable;
  }

  strncpy(work_buffer1_, root_directory_.c_str(), work_buffer1_len_);
  strncpy(work_buffer1_ + root_directory_.length(), payload->data_as_c_str(),
          work_buffer1_len_ - root_directory_.length());

#ifdef FTP_DEBUG
  LOGGER_INFO("FTP: open '%s'", work_buffer1_);
#endif

  struct stat st {};

  if (stat(work_buffer1_, &st) != 0) {
    // fail only if requested open for read
    if (oflag & O_RDONLY) {
      return kErrFailErrno;

    } else {
      st.st_size = 0;
    }
  }

  uint32_t fileSize = st.st_size;

  // Set mode to 666 incase oflag has O_CREAT
  int fd = ::open(work_buffer1_, oflag,
                  S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);

  if (fd < 0) {
    return kErrFailErrno;
  }

  session_info_.fd = fd;
  session_info_.file_size = fileSize;
  session_info_.stream_download = false;

  payload->session = 0;
  payload->size = sizeof(uint32_t);
  std::memcpy(payload->data, &fileSize, payload->size);

  return kErrNone;
}

/// @brief Responds to a Read command
FileServer::ErrorCode FileServer::WorkRead(Payload *payload) const {
  if (payload->session != 0 || session_info_.fd < 0) {
    return kErrInvalidSession;
  }

#ifdef FTP_DEBUG
  LOGGER_INFO("FTP: read offset:%d", payload->offset);
#endif

  // We have to test seek past EOF ourselves, lseek will allow seek past EOF
  if (payload->offset >= session_info_.file_size) {
    LOGGER_ERROR("request past EOF");
    return kErrEOF;
  }

  if (lseek(session_info_.fd, payload->offset, SEEK_SET) < 0) {
    LOGGER_ERROR("seek fail");
    return kErrFailErrno;
  }

  auto bytes_read = ::read(session_info_.fd, &payload->data[0], kMaxDataLength);

  if (bytes_read < 0) {
    // Negative return indicates error other than eof
    LOGGER_ERROR("read fail %ld", bytes_read);
    return kErrFailErrno;
  }

  payload->size = bytes_read;

  return kErrNone;
}

/// @brief Responds to a Stream command
FileServer::ErrorCode FileServer::WorkBurst(Payload *payload) {
  if (payload->session != 0 && session_info_.fd < 0) {
    return kErrInvalidSession;
  }

#ifdef FTP_DEBUG
  LOGGER_INFO("FTP: burst offset:%d", payload->offset);
#endif
  // Setup for streaming sends
  session_info_.stream_download = true;
  session_info_.stream_offset = payload->offset;
  session_info_.stream_chunk_transmitted = 0;
  session_info_.stream_seq_number = payload->seq_number + 1;

  return kErrNone;
}

/// @brief Responds to a Write command
FileServer::ErrorCode FileServer::WorkWrite(Payload *payload) const {
  if (payload->session != 0 && session_info_.fd < 0) {
    return kErrInvalidSession;
  }

  if (lseek(session_info_.fd, payload->offset, SEEK_SET) < 0) {
    // Unable to see to the specified location
    LOGGER_ERROR("seek fail");
    return kErrFailErrno;
  }

  auto bytes_written =
      ::write(session_info_.fd, &payload->data[0], payload->size);

  if (bytes_written < 0) {
    // Negative return indicates error other than eof
    LOGGER_ERROR("write fail %ld", bytes_written);
    return kErrFailErrno;
  }

  payload->size = sizeof(uint32_t);
  std::memcpy(payload->data, &bytes_written, payload->size);

  return kErrNone;
}

/// @brief Responds to a RemoveFile command
FileServer::ErrorCode FileServer::WorkRemoveFile(Payload *payload) {
  strncpy(work_buffer1_, root_directory_.c_str(), work_buffer1_len_);
  strncpy(work_buffer1_ + root_directory_.length(), payload->data_as_c_str(),
          work_buffer1_len_ - root_directory_.length());
  // ensure termination
  work_buffer1_[work_buffer1_len_ - 1] = '\0';

  if (unlink(work_buffer1_) == 0) {
    payload->size = 0;
    return kErrNone;

  } else {
    return kErrFailErrno;
  }
}

/// @brief Responds to a TruncateFile command
FileServer::ErrorCode FileServer::WorkTruncateFile(Payload *payload) {
  strncpy(work_buffer1_, root_directory_.c_str(), work_buffer1_len_);
  strncpy(work_buffer1_ + root_directory_.length(), payload->data_as_c_str(),
          work_buffer1_len_ - root_directory_.length());
  // ensure termination
  work_buffer1_[work_buffer1_len_ - 1] = '\0';
  payload->size = 0;

  int ret = truncate(work_buffer1_, payload->offset);

  if (ret == 0) {
    return kErrNone;
  }

  return kErrFailErrno;
}

/// @brief Responds to a Terminate command
FileServer::ErrorCode FileServer::WorkTerminate(Payload *payload) {
  if (payload->session != 0 || session_info_.fd < 0) {
    return kErrInvalidSession;
  }

  ::close(session_info_.fd);
  session_info_.fd = -1;
  session_info_.stream_download = false;

  payload->size = 0;

  return kErrNone;
}

/// @brief Responds to a Reset command
FileServer::ErrorCode FileServer::WorkReset(Payload *payload) {
  if (session_info_.fd != -1) {
    ::close(session_info_.fd);
    session_info_.fd = -1;
    session_info_.stream_download = false;
  }

  payload->size = 0;

  return kErrNone;
}

/// @brief Responds to a Rename command
FileServer::ErrorCode FileServer::WorkRename(Payload *payload) {
  char *ptr = payload->data_as_c_str();
  size_t oldpath_sz = strlen(ptr);

  if (oldpath_sz == payload->size) {
    // no newpath
    errno = EINVAL;
    return kErrFailErrno;
  }

  strncpy(work_buffer1_, root_directory_.c_str(), work_buffer1_len_);
  strncpy(work_buffer1_ + root_directory_.length(), ptr,
          work_buffer1_len_ - root_directory_.length());
  work_buffer1_[work_buffer1_len_ - 1] = '\0';  // ensure termination

  strncpy(work_buffer2_, root_directory_.c_str(), work_buffer2_len_);
  strncpy(work_buffer2_ + root_directory_.length(), ptr + oldpath_sz + 1,
          work_buffer2_len_ - root_directory_.length());
  work_buffer2_[work_buffer2_len_ - 1] = '\0';  // ensure termination

  if (rename(work_buffer1_, work_buffer2_) == 0) {
    payload->size = 0;
    return kErrNone;

  } else {
    return kErrFailErrno;
  }
}

/// @brief Responds to a RemoveDirectory command
FileServer::ErrorCode FileServer::WorkRemoveDirectory(Payload *payload) {
  strncpy(work_buffer1_, root_directory_.c_str(), work_buffer1_len_);
  strncpy(work_buffer1_ + root_directory_.length(), payload->data_as_c_str(),
          work_buffer1_len_ - root_directory_.length());
  // ensure termination
  work_buffer1_[work_buffer1_len_ - 1] = '\0';

  if (rmdir(work_buffer1_) == 0) {
    payload->size = 0;
    return kErrNone;

  } else {
    return kErrFailErrno;
  }
}

/// @brief Responds to a CreateDirectory command
FileServer::ErrorCode FileServer::WorkCreateDirectory(Payload *payload) {
  strncpy(work_buffer1_, root_directory_.c_str(), work_buffer1_len_);
  strncpy(work_buffer1_ + root_directory_.length(), payload->data_as_c_str(),
          work_buffer1_len_ - root_directory_.length());
  // ensure termination
  work_buffer1_[work_buffer1_len_ - 1] = '\0';

  if (mkdir(work_buffer1_, S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
    payload->size = 0;
    return kErrNone;

  } else {
    return kErrFailErrno;
  }
}

/// @brief Responds to a CalcFileCRC32 command
FileServer::ErrorCode FileServer::WorkCalcFileCrc32(Payload *payload) {
  strncpy(work_buffer2_, root_directory_.c_str(), work_buffer2_len_);
  strncpy(work_buffer2_ + root_directory_.length(), payload->data_as_c_str(),
          work_buffer2_len_ - root_directory_.length());
  // ensure termination
  work_buffer2_[work_buffer2_len_ - 1] = '\0';

  int fd = ::open(work_buffer2_, O_RDONLY);
  if (fd < 0) {
    return kErrFailErrno;
  }

  uint32_t checksum = 0;
  ssize_t bytes_read;
  do {
    bytes_read = ::read(fd, work_buffer2_, work_buffer2_len_);

    if (bytes_read < 0) {
      int r_errno = errno;
      ::close(fd);
      errno = r_errno;
      return kErrFailErrno;
    }

    checksum = uftp_crc32part((uint8_t *)work_buffer2_, bytes_read, checksum);
  } while (bytes_read == work_buffer2_len_);

  ::close(fd);

  payload->size = sizeof(uint32_t);
  std::memcpy(payload->data, &checksum, payload->size);
  return kErrNone;
}

/// @brief Copy file (with limited space)
int FileServer::CopyFile(const char *src_path, const char *dst_path,
                         size_t length) {
  auto src_fd = ::open(src_path, O_RDONLY);

  if (src_fd < 0) {
    return -1;
  }

  auto dst_fd = ::open(dst_path, O_CREAT | O_TRUNC | O_WRONLY
// POSIX requires the permissions to be supplied if O_CREAT passed
#ifdef __PX4_POSIX
                       ,
                       0666
#endif
  );

  int op_errno = 0;
  if (dst_fd < 0) {
    op_errno = errno;
    ::close(src_fd);
    errno = op_errno;
    return -1;
  }

  while (length > 0) {
    ssize_t bytes_read, bytes_written;
    size_t blen = (length > work_buffer2_len_) ? work_buffer2_len_ : length;

    bytes_read = ::read(src_fd, work_buffer2_, blen);

    if (bytes_read == 0) {
      // EOF
      break;

    } else if (bytes_read < 0) {
      LOGGER_ERROR("cp: read");
      op_errno = errno;
      break;
    }

    bytes_written = ::write(dst_fd, work_buffer2_, bytes_read);

    if (bytes_written != bytes_read) {
      LOGGER_ERROR("cp: short write");
      op_errno = errno;
      break;
    }

    length -= bytes_written;
  }

  ::close(src_fd);
  ::close(dst_fd);

  errno = op_errno;
  return (length > 0) ? -1 : 0;
}

void FileServer::Send() {
  if (work_buffer1_ || work_buffer2_) {
    // free the work buffers if they are not used for a while
    if ((absolute_time_us() - last_work_buffer_access_) > 2 * 1000 * 1000) {
      if (work_buffer1_) {
        delete[] work_buffer1_;
        work_buffer1_ = nullptr;
      }

      if (work_buffer2_) {
        delete[] work_buffer2_;
        work_buffer2_ = nullptr;
      }
    }

  } else if (session_info_.fd != -1) {
    // close session without activity
    if ((absolute_time_us() - last_work_buffer_access_) > 10 * 1000 * 1000) {
      ::close(session_info_.fd);
      session_info_.fd = -1;
      session_info_.stream_download = false;
      last_reply_valid_ = false;
      LOGGER_WARN("Session was closed without activity");
    }
  }

  // Anything to stream?
  if (!session_info_.stream_download) {
    return;
  }

  // Skip send if not enough room
  unsigned max_bytes_to_send = get_size();

#ifdef FTP_DEBUG
  LOGGER_INFO("MavlinkFTP::send max_bytes_to_send(%d)", max_bytes_to_send);
#endif

  if (max_bytes_to_send < get_size()) {
    return;
  }

  // Send stream packets until buffer is full

  bool more_data;

  do {
    more_data = false;

    ErrorCode error_code = kErrNone;

    uint8_t payload_buffer[sizeof(Payload) + kMaxDataLength];
    auto *payload = reinterpret_cast<Payload *>(&payload_buffer[0]);

    payload->seq_number = session_info_.stream_seq_number;
    payload->session = 0;
    payload->opcode = kRspAck;
    payload->req_opcode = kCmdBurstReadFile;
    payload->offset = session_info_.stream_offset;
    session_info_.stream_seq_number++;

#ifdef FTP_DEBUG
    LOGGER_INFO("stream send: offset %d", session_info_.stream_offset);
#endif

    // We have to test seek past EOF ourselves, lseek will allow seek past EOF
    if (session_info_.stream_offset >= session_info_.file_size) {
      error_code = kErrEOF;
#ifdef FTP_DEBUG
      LOGGER_INFO("stream download: sending Nak EOF");
#endif
    }

    if (error_code == kErrNone) {
      if (lseek(session_info_.fd, payload->offset, SEEK_SET) < 0) {
        error_code = kErrFailErrno;
#ifdef FTP_DEBUG
        LOGGER_WARN("stream download: seek fail");
#endif
      }
    }

    if (error_code == kErrNone) {
      auto bytes_read =
          ::read(session_info_.fd, &payload->data[0], kMaxDataLength);

      if (bytes_read < 0) {
        // Negative return indicates error other than eof
        error_code = kErrFailErrno;
#ifdef FTP_DEBUG
        LOGGER_WARN("stream download: read fail");
#endif

      } else {
        payload->size = bytes_read;
        session_info_.stream_offset += bytes_read;
        session_info_.stream_chunk_transmitted += bytes_read;
      }
    }

    if (error_code != kErrNone) {
      payload->opcode = kRspNak;
      payload->size = 1;
      uint8_t *pData = &payload->data[0];
      *pData = error_code;  // Straight reference to data[0] is causing bogus
                            // gcc array subscript error

      if (error_code == kErrFailErrno) {
        int r_errno = errno;
        payload->size = 2;
        payload->data[1] = r_errno;
      }

      session_info_.stream_download = false;

    } else {
#ifndef MAVLINK_FTP_UNIT_TEST

      if (max_bytes_to_send < (get_size() * 2)) {
        more_data = false;

        /* perform transfers in 35K chunks - this is determined empirical */
        if (session_info_.stream_chunk_transmitted > 35000) {
          payload->burst_complete = true;
          session_info_.stream_download = false;
          session_info_.stream_chunk_transmitted = 0;
        }

      } else {
#endif
        more_data = true;
        payload->burst_complete = false;
#ifndef MAVLINK_FTP_UNIT_TEST
        max_bytes_to_send -= get_size();
      }

#endif
    }

    Reply(payload);
  } while (more_data);
}

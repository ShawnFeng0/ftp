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
#include <vector>

#include "crc32.h"
#include "md5.h"
#include "ulog/ulog.h"

#define FTP_DEBUG

using namespace uftp;

FileServer::FileServer(std::string root_directory, void *user_data,
                       WriteCallback write_callback)
    : root_directory_(std::move(root_directory)),
      user_data_(user_data),
      write_callback_(write_callback) {
  // initialize session
  session_info_.fd = -1;
  burst_payload_.resize(sizeof(uftp::Payload) +
                        uftp::FileServer::kMaxDataLength);
}

FileServer::~FileServer() = default;

/// @brief Processes an FTP message
void FileServer::ProcessRequest(const Payload *payload_in) {
  bool stream_send = false;

  ErrorCode errorCode = kErrNone;

  std::unique_lock<std::mutex> lg(session_lock_);

  std::vector<uint8_t> payload_buffer;
  payload_buffer.resize(sizeof(Payload) + kMaxDataLength);
  memcpy(payload_buffer.data(), payload_in,
         std::min(payload_buffer.size(), sizeof(Payload) + payload_in->size));
  auto payload = reinterpret_cast<Payload *>(payload_buffer.data());

#ifdef FTP_DEBUG
  auto payload_str = payload->to_string();
  LOGGER_TOKEN(payload_str.c_str());
#endif

  // basic sanity checks; must validate length before use
  if (payload_in->size > kMaxDataLength) {
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

    case kCmdListDirectoryWithTimeInfo:
      errorCode = WorkListWithTimeInfo(payload);
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

    case kCmdCalcFileMD5:
      errorCode = WorkCalcFileMd5(payload);
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

/// @brief Sends the specified FTP response message out through mavlink
int FileServer::Reply(Payload *payload) {
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
//  LOGGER_INFO("FTP: %s seq_number: %d",
//              payload->opcode == kRspAck ? "Ack" : "Nak",
//              payload->seq_number);
#endif

  if (write_callback_) {
    return write_callback_(user_data_, reinterpret_cast<const char *>(payload),
                           sizeof(Payload) + payload->size);
  } else {
    return 0;
  }
}

/// @brief Responds to a List command
FileServer::ErrorCode FileServer::WorkList(Payload *payload) {
  std::string work_path{root_directory_ + payload->data_as_c_str()};

  ErrorCode errorCode = kErrNone;
  unsigned offset = 0;

  DIR *dp = opendir(work_path.c_str());

  if (dp == nullptr) {
    LOGGER_WARN("File open failed %s", work_path.c_str());
    return kErrFileNotFound;
  }

#ifdef FTP_DEBUG
  LOGGER_INFO("FTP: list %s offset %d", work_path.c_str(), payload->offset);
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
        std::string file_path = work_path + '/' + result->d_name;

        struct stat st {};
        if (stat(file_path.c_str(), &st) == 0) {
          fileSize = st.st_size;
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

    std::string file_path;

    if (direntType == kDirentSkip) {
      // Skip send only dirent identifier

    } else if (direntType == kDirentFile) {
      // Files send filename and file length
      file_path = std::string{result->d_name} + '\t' + std::to_string(fileSize);

    } else {
      // Everything else just sends name
      file_path = result->d_name;
    }

    size_t nameLen = file_path.length();

    // Do we have room for the name, the one char directory identifier and the
    // null terminator?
    if ((offset + nameLen + 2) > kMaxDataLength) {
      break;
    }

    // Move the data into the buffer
    payload->data[offset++] = direntType;
    strcpy((char *)&payload->data[offset], file_path.c_str());
#ifdef FTP_DEBUG
    LOGGER_INFO("FTP: list %s %s", work_path.c_str(),
                (char *)&payload->data[offset - 1]);
#endif
    offset += nameLen + 1;
  }

  closedir(dp);
  payload->size = offset;

  return errorCode;
}

/// @brief Responds to a List command
FileServer::ErrorCode FileServer::WorkListWithTimeInfo(Payload *payload) {
  std::string work_path{root_directory_ + payload->data_as_c_str()};

  ErrorCode errorCode = kErrNone;
  unsigned offset = 0;

  DIR *dp = opendir(work_path.c_str());

  if (dp == nullptr) {
    LOGGER_WARN("File open failed %s", work_path.c_str());
    return kErrFileNotFound;
  }

#ifdef FTP_DEBUG
  LOGGER_INFO("FTP: list %s offset %d", work_path.c_str(), payload->offset);
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

    std::string file_info;

    if (direntType == kDirentSkip) {
      // Skip send only dirent identifier

    } else if (direntType == kDirentFile) {
      file_info = work_path + '/' + result->d_name;

      struct stat st {};
      if (stat(file_info.c_str(), &st) == 0) {
        // Files send filename and file length
        file_info =
            std::string{result->d_name} + '\t' + std::to_string(st.st_size) +
            '\t' + std::to_string(st.st_atime) + '\t' +
            std::to_string(st.st_mtime) + '\t' + std::to_string(st.st_ctime);
      } else {
        // Everything else just sends name
        file_info = result->d_name;
      }

    } else {
      // Everything else just sends name
      file_info = result->d_name;
    }

    size_t nameLen = file_info.length();

    // Do we have room for the name, the one char directory identifier and the
    // null terminator?
    if ((offset + nameLen + 2) > kMaxDataLength) {
      break;
    }

    // Move the data into the buffer
    payload->data[offset++] = direntType;
    strcpy((char *)&payload->data[offset], file_info.c_str());
#ifdef FTP_DEBUG
    LOGGER_INFO("FTP: list %s %s", work_path.c_str(),
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

  std::string work_path{root_directory_ + payload->data_as_c_str()};

#ifdef FTP_DEBUG
  LOGGER_INFO("FTP: open '%s'", work_path.c_str());
#endif

  struct stat st {};

  if (stat(work_path.c_str(), &st) != 0) {
    // fail only if requested open for read
    if (oflag & O_RDONLY) {
      return kErrFailErrno;

    } else {
      st.st_size = 0;
    }
  }

  uint32_t fileSize = st.st_size;

  // Set mode to 666 incase oflag has O_CREAT
  int fd = ::open(work_path.c_str(), oflag,
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
  std::string work_path{root_directory_ + payload->data_as_c_str()};

  if (unlink(work_path.c_str()) == 0) {
    payload->size = 0;
    return kErrNone;

  } else {
    return kErrFailErrno;
  }
}

/// @brief Responds to a TruncateFile command
FileServer::ErrorCode FileServer::WorkTruncateFile(Payload *payload) {
  std::string work_path{root_directory_ + payload->data_as_c_str()};

  payload->size = 0;

  int ret = truncate(work_path.c_str(), payload->offset);

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
  const char *ptr = payload->data_as_c_str();
  size_t oldpath_sz = strlen(ptr);

  if (oldpath_sz == payload->size) {
    // no newpath
    errno = EINVAL;
    return kErrFailErrno;
  }

  std::string old_path{root_directory_ + ptr};
  std::string new_path{root_directory_ + std::string(ptr + oldpath_sz + 1)};

  if (rename(old_path.c_str(), new_path.c_str()) == 0) {
    payload->size = 0;
    return kErrNone;

  } else {
    return kErrFailErrno;
  }
}

/// @brief Responds to a RemoveDirectory command
FileServer::ErrorCode FileServer::WorkRemoveDirectory(Payload *payload) {
  std::string work_path{root_directory_ + payload->data_as_c_str()};

  if (rmdir(work_path.c_str()) == 0) {
    payload->size = 0;
    return kErrNone;

  } else {
    return kErrFailErrno;
  }
}

/// @brief Responds to a CreateDirectory command
FileServer::ErrorCode FileServer::WorkCreateDirectory(Payload *payload) {
  std::string work_path{root_directory_ + payload->data_as_c_str()};

  if (mkdir(work_path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
    payload->size = 0;
    return kErrNone;

  } else {
    return kErrFailErrno;
  }
}

/// @brief Responds to a CalcFileCRC32 command
FileServer::ErrorCode FileServer::WorkCalcFileCrc32(Payload *payload) {
  std::string work_path{root_directory_ + payload->data_as_c_str()};

  int fd = ::open(work_path.c_str(), O_RDONLY);
  if (fd < 0) {
    return kErrFailErrno;
  }

  std::vector<uint8_t> buffer;
  buffer.resize(256);

  uint32_t checksum = 0;
  ssize_t bytes_read;
  do {
    bytes_read = ::read(fd, buffer.data(), buffer.size());

    if (bytes_read < 0) {
      int r_errno = errno;
      ::close(fd);
      errno = r_errno;
      return kErrFailErrno;
    }

    checksum = uftp_crc32part(buffer.data(), bytes_read, checksum);
  } while (bytes_read == buffer.size());

  ::close(fd);

  payload->size = sizeof(uint32_t);
  std::memcpy(payload->data, &checksum, payload->size);
  return kErrNone;
}

/// @brief Responds to a CalcFileMd5 command
FileServer::ErrorCode FileServer::WorkCalcFileMd5(Payload *payload) {
  std::string work_path{root_directory_ + payload->data_as_c_str()};

  int fd = ::open(work_path.c_str(), O_RDONLY);
  if (fd < 0) {
    return kErrFailErrno;
  }

  std::vector<uint8_t> buffer;
  buffer.resize(256);

  ssize_t bytes_read;
  MD5 md5;
  do {
    bytes_read = ::read(fd, buffer.data(), buffer.size());

    if (bytes_read < 0) {
      int r_errno = errno;
      ::close(fd);
      errno = r_errno;
      return kErrFailErrno;
    }

    md5.Update(buffer.data(), bytes_read);
  } while (bytes_read == buffer.size());

  ::close(fd);

  char md5_str[33];
  md5.ToString(md5_str);

  payload->size = sizeof(md5_str);
  std::memcpy(payload->data, md5_str, payload->size);
  return kErrNone;
}

void FileServer::ProcessSend(size_t max_frames) {
  std::unique_lock<std::mutex> lg(session_lock_);

  // Anything to stream?
  if (!session_info_.stream_download) {
    return;
  }

  // Send stream packets until buffer is full
  bool more_data;

#ifdef FTP_DEBUG
  LOGGER_INFO("stream send: offset %d", session_info_.stream_offset);
#endif

  do {
    more_data = false;

    ErrorCode error_code = kErrNone;
    auto payload = reinterpret_cast<uftp::Payload *>(burst_payload_.data());
    payload->seq_number = session_info_.stream_seq_number;
    payload->session = 0;
    payload->opcode = kRspAck;
    payload->req_opcode = kCmdBurstReadFile;
    payload->offset = session_info_.stream_offset;
    payload->burst_complete = false;  // Not use
    session_info_.stream_seq_number++;

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

      Reply(payload);

      session_info_.stream_download = false;

    } else {
      int send_length = Reply(payload);

      // Sending failed, there is a problem, the session should be ended
      if (send_length < 0) {
        session_info_.stream_download = false;
#ifdef FTP_DEBUG
        LOGGER_WARN("stream download: send fail");
#endif

      } else if (send_length == 0) {
        // Sending failed, but not an error, try again next time
        more_data = false;
      } else {
        // Send successfully, continue to send
        more_data = true;
        session_info_.stream_offset += payload->size;
      }
    }
  } while (more_data && max_frames--);

#ifdef FTP_DEBUG
  LOGGER_INFO("stream send over: offset %d", session_info_.stream_offset);
#endif
}

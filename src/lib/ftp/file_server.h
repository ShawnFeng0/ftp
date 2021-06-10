#pragma once

#include <dirent.h>

#include <cstdint>
#include <ctime>

namespace uftp {

/// @brief This is the payload which is in
/// mavlink_file_transfer_protocol_t.payload. This needs to be packed, because
/// it's typecasted from mavlink_file_transfer_protocol_t.payload, which
/// starts at a 3 byte offset, causing an unaligned access to seq_number and
/// offset
struct Payload {
  uint32_t size;        ///< Size of data
  uint32_t offset;      ///< Offsets for List and Read commands
  uint16_t seq_number;  ///< sequence number for message
  uint8_t session;      ///< Session id for read and write commands
  uint8_t opcode;       ///< Command opcode
  uint8_t req_opcode;   ///< Request opcode returned in kRspAck, kRspNak message
  uint8_t burst_complete;  ///< Only used if req_opcode=kCmdBurstReadFile - 1:
                           ///< set of burst packets complete, 0: More burst
                           ///< packets coming.
  uint8_t data[];          ///< command data, varies by Opcode
};

class FileServer {
 public:
  typedef size_t (*WriteCallback)(void *user_data, const char *data,
                                  size_t len);

  explicit FileServer(void *user_data, WriteCallback write_callback);
  ~FileServer();

  /**
   * Handle sending of messages. Call this regularly at a fixed frequency.
   * @param t current time
   */
  void Send();
  void ProcessRequest(Payload *payload);

  /// @brief Command opcodes
  enum Opcode : uint8_t {
    kCmdNone,              ///< ignored, always acked
    kCmdTerminateSession,  ///< Terminates open Read session
    kCmdResetSessions,     ///< Terminates all open Read sessions
    kCmdListDirectory,     ///< List files in <path> from <offset>
    kCmdOpenFileRO,  ///< Opens file at <path> for reading, returns <session>
    kCmdReadFile,    ///< Reads <size> bytes from <offset> in <session>
    kCmdCreateFile,  ///< Creates file at <path> for writing, returns <session>
    kCmdWriteFile,   ///< Writes <size> bytes to <offset> in <session>
    kCmdRemoveFile,  ///< Remove file at <path>
    kCmdCreateDirectory,  ///< Creates directory at <path>
    kCmdRemoveDirectory,  ///< Removes Directory at <path>, must be empty
    kCmdOpenFileWO,     ///< Opens file at <path> for writing, returns <session>
    kCmdTruncateFile,   ///< Truncate file at <path> to <offset> length
    kCmdRename,         ///< Rename <path1> to <path2>
    kCmdCalcFileCRC32,  ///< Calculate CRC32 for file at <path>
    kCmdBurstReadFile,  ///< Burst download session file

    kRspAck = 128,  ///< Ack response
    kRspNak         ///< Nak response
  };

  /// @brief Error codes returned in Nak response PayloadHeader.data[0].
  enum ErrorCode : uint8_t {
    kErrNone,
    kErrFail,                 ///< Unknown failure
    kErrFailErrno,            ///< Command failed, errno sent back in
                              ///< PayloadHeader.data[1]
    kErrInvalidDataSize,      ///< PayloadHeader.size is invalid
    kErrInvalidSession,       ///< Session is not currently open
    kErrNoSessionsAvailable,  ///< All available Sessions in use
    kErrEOF,             ///< Offset past end of file for List and Read commands
    kErrUnknownCommand,  ///< Unknown command opcode
    kErrFailFileExists,  ///< File/directory exists already
    kErrFailFileProtected,  ///< File/directory is write protected
    kErrFileNotFound        ///< File/directory not found
  };

  unsigned get_size() const;

 private:
  static char *data_as_cstring(Payload *payload);

  void Reply(Payload *payload);

  int CopyFile(const char *src_path, const char *dst_path, size_t length);

  ErrorCode WorkList(Payload *payload);
  ErrorCode WorkOpen(Payload *payload, int oflag);
  ErrorCode WorkRead(Payload *payload) const;
  ErrorCode WorkBurst(Payload *payload);
  ErrorCode WorkWrite(Payload *payload) const;
  ErrorCode WorkTerminate(Payload *payload);
  ErrorCode WorkReset(Payload *payload);
  ErrorCode WorkRemoveDirectory(Payload *payload);
  ErrorCode WorkCreateDirectory(Payload *payload);
  ErrorCode WorkRemoveFile(Payload *payload);
  ErrorCode WorkTruncateFile(Payload *payload);
  ErrorCode WorkRename(Payload *payload);
  ErrorCode WorkCalcFileCrc32(Payload *payload);

  /**
   * make sure that the working buffers _work_buffer* are allocated
   * @return true if buffers exist, false if allocation failed
   */
  bool ensure_buffers_exist();

  static const char kDirentFile =
      'F';  ///< Identifies File returned from List command
  static const char kDirentDir =
      'D';  ///< Identifies Directory returned from List command
  static const char kDirentSkip =
      'S';  ///< Identifies Skipped entry from List command

  static constexpr size_t kMaxPacketLength = 251;

  /// @brief Maximum data size in RequestHeader::data
  static constexpr size_t kMaxDataLength = kMaxPacketLength - sizeof(Payload);

  struct SessionInfo {
    int fd;
    uint32_t file_size;
    bool stream_download;
    uint32_t stream_offset;
    uint16_t stream_seq_number;
    unsigned stream_chunk_transmitted;
  } session_info_;  ///< Session info, fd=-1 for no active session

  void *user_data_;
  WriteCallback write_callback_;

  /* do not allow copying this class */
  FileServer(const FileServer &);
  FileServer operator=(const FileServer &);

  /* work buffers: they're allocated as soon as we get the first request (lazy,
   * since FTP is rarely used) */
  char *work_buffer1_{nullptr};
  static constexpr int work_buffer1_len_ = kMaxDataLength;
  char *work_buffer2_{nullptr};
  static constexpr int work_buffer2_len_ = 256;
  uint64_t last_work_buffer_access_{
      0};  ///< timestamp when the buffers were last accessed

  // prepend a root directory to each file/dir access to avoid enumerating the
  // full FS tree (e.g. on Linux). Note that requests can still fall outside of
  // the root dir by using ../..
  static constexpr const char root_dir_[] = "/";
  static constexpr const int root_dir_len_ = sizeof(root_dir_) - 1;

  bool last_reply_valid_ = false;
  uint8_t last_reply_[sizeof(Payload) + sizeof(uint32_t)]{};
};

}  // namespace uftp

#include <ftp/file_server.h>
#include <ftp/md5.h>
#include <ulog/ulog.h>

#include <iostream>
#include <vector>

std::vector<uint8_t> GeneratePayloadBuffer(uint32_t size, uint32_t offset,
                                           uint16_t seq_number, uint8_t session,
                                           uint8_t opcode, uint8_t req_opcode,
                                           uint8_t burst_complete,
                                           const std::string &data) {
  std::vector<uint8_t> payload_vector;
  payload_vector.resize(sizeof(uftp::Payload) +
                        uftp::FileServer::kMaxDataLength);
  auto payload = reinterpret_cast<uftp::Payload *>(payload_vector.data());
  payload->size = size;
  payload->offset = offset;
  payload->seq_number = seq_number;
  payload->session = session;
  payload->opcode = opcode;
  payload->req_opcode = req_opcode;
  payload->burst_complete = burst_complete;
  snprintf((char *)(payload->data), uftp::FileServer::kMaxDataLength, "%s",
           data.c_str());
  return payload_vector;
}

int main() {
  uftp::FileServer server{
      "", nullptr, [](void *user_data, const char *data, size_t len) -> int {
        LOGGER_HEX_DUMP(data, len, 16);
        return (int)len;
      }};

  std::string directory{"."};

  auto payload_vector = GeneratePayloadBuffer(
      directory.size() + 1, 0, 0, 0,
      uftp::FileServer::Opcode::kCmdListDirectoryWithTimeInfo, 0, 0, directory);
  server.ProcessRequest(
      reinterpret_cast<uftp::Payload *>(payload_vector.data()));
  server.ProcessSend(10);

  uftp::MD5 md5;
  std::string str{
      ";lkjasdfl;kjaskfl;djas;djawerpiuqwerpouasdklvzxvlasfjlqwerijl"};
  md5.Update(reinterpret_cast<const uint8_t *>(str.data()), str.size());
  char out[33];
  md5.ToString(out);
  LOGGER_TOKEN(out);

  if (std::string{"72d308a8b4a438f6e97ebff6743b034e"} != out) {
    LOGGER_ERROR("md5 error");
  }
  return 0;
}

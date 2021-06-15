#include <ftp/file_server.h>
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
      "", nullptr, [](void *user_data, const char *data, size_t len) -> size_t {
        LOGGER_HEX_DUMP(data, len, 16);
        return len;
      }};

  std::string directory{"."};

  auto payload_vector = GeneratePayloadBuffer(
      directory.size() + 1, 0, 0, 0,
      uftp::FileServer::Opcode::kCmdListDirectoryWithTimeInfo, 0, 0, directory);
  server.ProcessRequest(
      reinterpret_cast<uftp::Payload *>(payload_vector.data()));
  server.Send();
  return 0;
}

#include <memory>
#include <string>
#include <utility>
#include "third_party/httplib.h"
#include "third_party/json.hpp"

using json = nlohmann::json;

const std::string API_URL = "http://cppmiltech.com.ua";
const std::string API_KEY = "dz12-vX7mK4qT9r2w";

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class HttpClient {
public:
  HttpClient(std::string& studentId)
    : client(std::make_unique<httplib::Client>(API_URL))
    , studentId(std::move(studentId))
  {
    client->set_connection_timeout(2, 0);
    client->set_read_timeout(2, 0);
  };

  bool sendResults(const int testId) const
  {
    json body;

    body["studentId"] = studentId;
    body["testId"] = testId;
    body["simulation"] = "";

    return false;
  }

  virtual ~HttpClient() = default;

private:
  std::unique_ptr<httplib::Client> client;
  std::string studentId;
};

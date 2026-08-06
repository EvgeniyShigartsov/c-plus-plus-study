#include <memory>
#include <string>
#include <utility>
#include "third_party/httplib.h"
#include "third_party/json.hpp"

using json = nlohmann::json;

const std::string API_URL = "http://cppmiltech.com.ua";
const std::string API_KEY = "dz12-vX7mK4qT9r2w";

enum class SendOutcome {
  Success,           // 2xx успіх
  PermanentFailure,  // 4xx, 400/401 - повторювати немає сенсу
  RetryableFailure,  // 503 або таймаут - можна повторити
};

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

  SendOutcome sendResults(const std::string& testId, const json& simulationJson) const
  {
    json body;

    body["studentId"] = studentId;
    body["testId"] = testId;
    body["simulation"] = simulationJson;

    const httplib::Headers headers = {{"x-api-key", API_KEY}};

    const httplib::Result res = client->Post("/api/dz12/results", headers, body.dump(), "application/json");

    if (!res) {
      return SendOutcome::RetryableFailure;
    }

    if (res->status == 200 || res->status == 201) {
      return SendOutcome::Success;
    }

    if (res->status == 503) {
      return SendOutcome::RetryableFailure;
    }

    return SendOutcome::PermanentFailure;
  }

  virtual ~HttpClient() = default;

private:
  std::unique_ptr<httplib::Client> client;
  std::string studentId;
};

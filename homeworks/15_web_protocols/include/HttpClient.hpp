#pragma once

#include <memory>
#include <string>
#include "third_party/httplib.h"
#include "third_party/json.hpp"

using json = nlohmann::json;

enum class SendOutcome {
  Success,           // 2xx успіх
  PermanentFailure,  // 4xx, 400/401 - повторювати немає сенсу
  RetryableFailure,  // 503 або таймаут - можна повторити
};

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class HttpClient {
public:
  HttpClient(std::string& studentId);

  SendOutcome sendResults(const std::string& testId, const json& simulationJson) const;
  bool ensureResults(const std::string& testId) const;

  virtual ~HttpClient() = default;

private:
  std::unique_ptr<httplib::Client> client;
  std::string studentId;
  const httplib::Headers headers;
};

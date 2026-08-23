#pragma once

#include <memory>
#include <string>
#include "third_party/httplib.h"
#include "third_party/json.hpp"

using json = nlohmann::json;

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class HttpClient {
public:
  HttpClient(const std::string& studentId);

  httplib::Result sendResults(const std::string& testId, const json& simulationJson) const;
  bool ensureResults(const std::string& testId) const;

  virtual ~HttpClient() = default;

private:
  std::unique_ptr<httplib::Client> client;
  std::string studentId;
  const httplib::Headers headers;
};

#include "HttpClient.hpp"

#include <utility>

const std::string API_URL = "http://cppmiltech.com.ua";

HttpClient::HttpClient(std::string& studentId)
  : client(std::make_unique<httplib::Client>(API_URL))
  , studentId(std::move(studentId))
  , headers({{"x-api-key", "dz12-vX7mK4qT9r2w"}})
{
  client->set_connection_timeout(2, 0);
  client->set_read_timeout(2, 0);
}

SendOutcome HttpClient::sendResults(const std::string& testId, const json& simulationJson) const
{
  json body;

  body["studentId"] = studentId;
  body["testId"] = testId;
  body["simulation"] = simulationJson;

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

bool HttpClient::ensureResults(const std::string& testId) const
{
  const std::string url = "/api/dz12/results/" + testId + "/" + studentId;

  const httplib::Result res = client->Get(url, headers);

  if (!res || res->status != 200) {
    return false;
  }

  const json body = json::parse(res->body, nullptr, false);

  if (body.is_discarded()) {
    return false;
  }

  return body.value("found", false);
}

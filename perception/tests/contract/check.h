// Minimal check harness shared by the contract-level tests.
//
// The repository has no gtest (dpcbf_safety_filter_test is a plain executable registered
// with CTest), so this mirrors the counting/reporting style the existing perception tests
// already use, factored out because two binaries now need it.
#ifndef PERCEPTION_TESTS_CONTRACT_CHECK_H_
#define PERCEPTION_TESTS_CONTRACT_CHECK_H_

#include <cstdio>
#include <cstring>
#include <string>

namespace perception_test {

inline int g_checks = 0;
inline int g_failures = 0;

inline void Check(bool condition, const std::string& name, const std::string& detail = "") {
  ++g_checks;
  if (condition) {
    std::printf("  PASS  %s%s%s\n", name.c_str(), detail.empty() ? "" : "  ", detail.c_str());
  } else {
    ++g_failures;
    std::printf("  FAIL  %s%s%s\n", name.c_str(), detail.empty() ? "" : "  ", detail.c_str());
  }
}

// A contract is valid: Validate() returned nullptr.
template <typename Contract>
void CheckValid(const Contract& contract, const std::string& name) {
  const char* reason = contract.Validate();
  Check(reason == nullptr, name, reason == nullptr ? "" : std::string("got: ") + reason);
}

// A contract is invalid, AND for the stated reason. Matching on a substring rather than
// only on "some error" is deliberate: a validation test that accepts any failure passes
// just as happily when an unrelated field is what actually broke.
template <typename Contract>
void CheckInvalid(const Contract& contract, const std::string& expected_substring,
                  const std::string& name) {
  const char* reason = contract.Validate();
  if (reason == nullptr) {
    Check(false, name, "expected a validation failure, got none");
    return;
  }
  const bool matches = std::strstr(reason, expected_substring.c_str()) != nullptr;
  Check(matches, name, matches ? "" : std::string("got: ") + reason);
}

inline void Section(const char* title) { std::printf("\n-- %s\n", title); }

inline int Report(const char* suite) {
  std::printf("\n%s: %d/%d checks passed\n", suite, g_checks - g_failures, g_checks);
  if (g_failures != 0) {
    std::printf("RESULT: FAIL (%d)\n", g_failures);
    return 1;
  }
  std::printf("RESULT: PASS\n");
  return 0;
}

}  // namespace perception_test

#endif  // PERCEPTION_TESTS_CONTRACT_CHECK_H_

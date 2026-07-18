// Single entry point for ArcCollision's native (C/C++) algorithm tests. Runs every
// test suite in turn, prints a pass/fail line for each, and returns 0 only if all
// pass -- so it works both as a plain executable you can run directly and as a
// CTest command. Each suite returns 0 on success or a small nonzero diagnostic
// code on the first failure (see the individual test sources for the codes).

#include <cstdio>

extern "C" int arc_run_c_api_smoke(void);
extern "C" int arc_run_simd_integer_tests(void);

namespace {

struct Suite {
    const char* name;
    int (*run)(void);
};

const Suite kSuites[] = {
    {"c_api_smoke", arc_run_c_api_smoke},
    {"simd_integer_tests", arc_run_simd_integer_tests},
};

} // namespace

int main() {
    const int total = static_cast<int>(sizeof(kSuites) / sizeof(kSuites[0]));
    int passed = 0;
    for (const Suite& suite : kSuites) {
        const int code = suite.run();
        if (code == 0) {
            std::printf("[ PASS ] %s\n", suite.name);
            ++passed;
        } else {
            std::printf("[ FAIL ] %s (code %d)\n", suite.name, code);
        }
    }
    std::printf("\n%d/%d native test suites passed.\n", passed, total);
    return passed == total ? 0 : 1;
}

#include "satellite.hpp"
#include <stdexcept>
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>

int main()
{

  assert(dso::translate_satid("ja3") == dso::SATELLITE::JASON3);
  assert(dso::translate_satid("    ja3") == dso::SATELLITE::JASON3);
  assert(dso::translate_satid("ja3ksjdhfkjsdhf") == dso::SATELLITE::JASON3);
  assert(dso::translate_satid("  ja3 sdkfj") == dso::SATELLITE::JASON3);
  assert(dso::translate_satid("  ja33sdkfj") == dso::SATELLITE::JASON3);

  assert(dso::translate_satid("ja2") == dso::SATELLITE::JASON2);
  assert(dso::translate_satid("    ja2") == dso::SATELLITE::JASON2);
  assert(dso::translate_satid("ja2ksjdhfkjsdhf") == dso::SATELLITE::JASON2);
  assert(dso::translate_satid("  ja2 sdkfj") == dso::SATELLITE::JASON2);
  assert(dso::translate_satid("  ja22sdkfj") == dso::SATELLITE::JASON2);

  assert(dso::translate_satid("s3a") == dso::SATELLITE::SENTINEL3A);
  assert(dso::translate_satid("    s3a") == dso::SATELLITE::SENTINEL3A);
  assert(dso::translate_satid("s3aksjdhfkjsdhf") == dso::SATELLITE::SENTINEL3A);
  assert(dso::translate_satid("  s3a sdkfj") == dso::SATELLITE::SENTINEL3A);
  assert(dso::translate_satid("  s3a2sdkfj") == dso::SATELLITE::SENTINEL3A);

  assert(dso::translate_satid("s3b") == dso::SATELLITE::SENTINEL3B);
  assert(dso::translate_satid("    s3b") == dso::SATELLITE::SENTINEL3B);
  assert(dso::translate_satid("s3bksjdhfkjsdhf") == dso::SATELLITE::SENTINEL3B);
  assert(dso::translate_satid("  s3b sdkfj") == dso::SATELLITE::SENTINEL3B);
  assert(dso::translate_satid("  s3b2sdkfj") == dso::SATELLITE::SENTINEL3B);

  assert(dso::translate_satid("s6a") == dso::SATELLITE::SENTINEL6A);
  assert(dso::translate_satid("    s6a") == dso::SATELLITE::SENTINEL6A);
  assert(dso::translate_satid("s6aksjdhfkjsdhf") == dso::SATELLITE::SENTINEL6A);
  assert(dso::translate_satid("  s6a sdkfj") == dso::SATELLITE::SENTINEL6A);
  assert(dso::translate_satid("  s6a2sdkfj") == dso::SATELLITE::SENTINEL6A);

  const char *err[] = {
      "jja3", "jaa2", "sa3a", "3ja", "ss6a"};
  for (int i = 0; i < 5; i++)
  {
    try
    {
      dso::translate_satid(err[i]);
      return 3;
    }
    catch (std::runtime_error &)
    {
      ;
    }
  }

  return 0;
}

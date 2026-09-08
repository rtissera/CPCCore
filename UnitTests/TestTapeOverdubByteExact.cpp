#include "gtest/gtest.h"

#include "TestUtils.h"

#include <vector>

// Answers, directly and in-memory (no CSW export/reload step -- a separate
// null experiment in TestTapeFluxExact.cpp already proved the CSW format
// itself re-splits/re-frames entries on export/reload even with zero bugs
// involved, so "byte identical" can only be checked meaningfully against
// the live tape_array_, before any file round trip):
//
//   after a short, bounded overdub burst, is every flux entry that the
//   burst did NOT touch still EXACTLY (length, high) what it was before
//   the burst -- not shrunk, not corrupted, not wrapped?
//
// This is precisely what Bug 2's fix guarantees and what its two bugs
// violated: the "shorten now the following one" block ran on every
// recording tick regardless of how far from the write position an entry
// sat, once the loop's own `i` advanced past small already-consumed
// entries.
TEST(TapeOverdubByteExact, EntriesFarFromOverdubBurstAreUnchanged)
{
   DirectoriesImp dirImp;
   CDisplay display;
   Log log;
   SoundFactory soundFactory;
   ConfigurationManager conf_manager;
   EmulatorEngine machine;

   display.Init(false);
   display.Show(false);

   machine.SetDirectories(&dirImp);
   machine.SetLog(&log);
   machine.SetConfigurationManager(&conf_manager);
   machine.Init(&display, &soundFactory);
   machine.GetMem()->Initialisation();
   machine.LoadConfiguration("./TestConf.ini", "./TestConf_0.ini");
   machine.Reinit();

   srand(0xE7123456);
   machine.SetFixedSpeed(true);
   machine.SetSpeedLimit(EmulatorEngine::E_FULL);

   machine.LoadTape("./res/Tape/Lemmings (UK) (1991) (01. Level 01 FUN - JUST DIG!) (Version Split) [Original] [TAPE].cdt");
   for (int i = 0; i < 100; ++i)
      machine.RunTimeSlice();

   CTape* tape = machine.GetTape();
   PPI8255* ppi = machine.GetPPI();
   ASSERT_NE(nullptr, tape);
   ASSERT_NE(nullptr, ppi);
   ASSERT_GT(tape->GetNbInversions(), 300u) << "tape did not actually load, test would prove nothing";

   // Snapshot the WHOLE array before touching anything -- this is the
   // real, already-loaded commercial tape content ("written to tape"
   // side of the user's question), byte for byte.
   unsigned int nb_before = tape->GetNbInversions();
   std::vector<CTape::DebugFlux> before(nb_before);
   for (unsigned int i = 0; i < nb_before; ++i)
      ASSERT_TRUE(tape->GetFlux(i, before[i]));

   tape->Rewind();
   tape->SetMotorOn(true);
   tape->Record();

   int settle_ticks = 0;
   while (!tape->IsRecordOn() && settle_ticks < 10000)
   {
      tape->Tick();
      ++settle_ticks;
   }
   ASSERT_TRUE(tape->IsRecordOn()) << "recording never actually started";

   unsigned int overdub_start_idx = tape->GetTapePosition();

   // A short, bounded overdub burst: hold the write line constant for a
   // few thousand ticks (this_tick_time_ is always 4 while recording),
   // then stop. Real commercial pilot-tone/data-cell entries are far
   // longer than a handful of ticks, so this should only ever reach into
   // entries immediately around the write position -- everything beyond
   // a generous safety margin should be provably untouched.
   // Needs to run long enough to actually fully consume at least one
   // ahead-of-position entry and cross into the next one -- that crossing
   // (the loop's `i` advancing past 1) is exactly where the two stacked
   // bugs diverge from correct behaviour; a short burst that only ever
   // partially eats a single large entry never reaches it and would pass
   // identically whether the fix is present or not.
   ppi->tape_write_data_level_ = false;
   for (int i = 0; i < 300000; ++i)
      tape->Tick();
   tape->StopRecord();

   unsigned int nb_after = tape->GetNbInversions();

   // A single genuine level transition at the very start of the burst
   // legitimately splices in one new entry (the same real mechanism any
   // recording tick uses, not a bug) -- that shifts every later index by
   // a fixed delta. Comparing before[idx] against after[idx] directly
   // (ignoring this) produces a cascade of spurious "polarity changed"
   // mismatches that are really just an off-by-N alignment error, not
   // corruption. Compare by that delta instead.
   long long delta = (long long)nb_after - (long long)nb_before;
   fprintf(stderr, "DIAG: nb_before=%u nb_after=%u delta=%lld overdub_start_idx=%u\n", nb_before, nb_after, delta, overdub_start_idx);

   // Primary assertion, matching the actual bug signature directly: the
   // two stacked bugs produce a uint64_t WRAPAROUND (18446744073708754220
   // in the original repro), not a zero -- an entry legitimately consumed
   // by overdubbing gets set to 0, so "any nonzero length" cannot be used
   // to detect corruption (that was this test's own first, wrong attempt:
   // a "skip past a run of nonzero entries" heuristic walked straight
   // past the corrupted entry because a wrapped-huge value reads as
   // "nonzero" exactly like an ordinary untouched one). No real flux
   // pulse is anywhere near this long -- even a full 20-real-minute blank
   // tape span (InsertBlankTape's own default) is ~4.8e9 ticks, three
   // orders of magnitude below this bound, but light-years short of a
   // 64-bit wraparound.
   const unsigned long long kSaneMaxLength = 1000000000ULL; // 1e9 ticks (~250s of tape) -- generous, still nowhere near 2^64
   unsigned long long max_length_seen = 0;
   unsigned int max_length_idx = 0;
   for (unsigned int idx = 0; idx < nb_after; ++idx)
   {
      CTape::DebugFlux e;
      ASSERT_TRUE(tape->GetFlux(idx, e));
      if (e.length > max_length_seen)
      {
         max_length_seen = e.length;
         max_length_idx = idx;
      }
   }
   fprintf(stderr, "DIAG: max entry length after overdub = %llu at idx %u\n", max_length_seen, max_length_idx);
   EXPECT_LT(max_length_seen, kSaneMaxLength)
      << "entry " << max_length_idx << " has length " << max_length_seen
      << " -- classic uint64_t underflow signature (a legitimately-consumed entry is set to exactly 0, never wraps)";

   // Secondary assertion: entries well past the write region, matched by
   // the splice delta above, must be byte-identical to what they were
   // before the overdub -- proves damage from the fix's own bookkeeping
   // never bleeds further than the burst itself.
   const unsigned int kSafetyMargin = 2000; // entries -- comfortably past anything a 300000-tick burst could reach
   unsigned int untouched_start = overdub_start_idx + kSafetyMargin;
   ASSERT_LT(untouched_start, nb_before) << "test tape too short to leave an untouched tail -- pick a longer fixture or shorter burst";

   unsigned int checked = 0;
   for (unsigned int idx = untouched_start; idx < nb_before; ++idx)
   {
      long long after_idx = (long long)idx + delta;
      if (after_idx < 0 || after_idx >= (long long)nb_after) break;
      CTape::DebugFlux after_entry;
      ASSERT_TRUE(tape->GetFlux((unsigned int)after_idx, after_entry));
      EXPECT_EQ(before[idx].length, after_entry.length)
         << "entry " << idx << " length changed from " << before[idx].length
         << " to " << after_entry.length << " despite being well outside the overdub burst"
         << " -- corruption bled outside the write region";
      EXPECT_EQ(before[idx].high, after_entry.high)
         << "entry " << idx << " polarity changed despite being outside the overdub burst";
      ++checked;
   }
   EXPECT_GT(checked, 0u) << "no entries were actually compared -- test would prove nothing";
   fprintf(stderr, "DIAG: compared %u untouched entries (before-idx %u..%u) byte-for-byte against after-idx+delta, all must be identical to pre-overdub content\n",
      checked, untouched_start, untouched_start + checked - 1);
}

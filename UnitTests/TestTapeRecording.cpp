#include "gtest/gtest.h"

#include "TestUtils.h"

// Real crash found while wiring experimental tape-record support in
// SugarLibRetro (github.com/rtissera/SugarLibRetro): recording onto a
// genuinely blank tape (InsertBlankTape()) segfaults inside CTape::Tick(),
// once the tape's initial silent span runs out and Record()'s deferred
// start-of-recording code (the `start_record_` block) actually engages.
// `nb_inversions_`/`tape_position_` are unsigned, and the array-growth
// `memmove` calls compute their length as
//   nb_inversions_ - (tape_position_ + N)
// which wraps to a huge value whenever the tape doesn't yet have N more
// entries beyond the current position -- exactly the case right as
// recording starts on fresh blank media.
//
// The real-hardware repro needs the emulated tape to actually run out its
// (normally 20 real-minute) blank span, which took ~100 seconds of real
// emulated CPC time chasing this by hand in RetroArch. InsertBlankTape()
// now takes an optional duration so a test can shrink that to a handful of
// Tick() calls instead.
TEST(TapeRecording, RecordOntoFreshBlankTapeDoesNotCrash)
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

   CTape* tape = machine.GetTape();
   PPI8255* ppi = machine.GetPPI();
   ASSERT_NE(nullptr, tape);
   ASSERT_NE(nullptr, ppi);

   // Short blank span (1000 ticks instead of 20 real minutes) so it runs
   // out almost immediately -- this is what actually reaches the buggy
   // code path quickly instead of needing ~100 seconds of real emulated
   // time like the original bug report.
   tape->InsertBlankTape(1000);
   tape->Rewind();
   tape->SetMotorOn(true);
   tape->Record();

   // Simulate the guest toggling the cassette write line and cycling the
   // motor on/off, the way a real game's own tape-loading ROM routine does
   // while searching for a header that will never come on blank media --
   // the original bug report needed ~1.5 million inversions of exactly
   // this kind of incidental activity before it crashed, not a single
   // clean transition.
   unsigned int rng = 0x12345678u;
   bool level = false;
   for (int i = 0; i < 500000; ++i)
   {
      rng = rng * 1664525u + 1013904223u; // classic LCG, deterministic
      if ((rng & 0x7) == 0) level = !level;
      ppi->tape_write_data_level_ = level;
      if ((rng & 0x3FF) == 0)
      {
         tape->SetMotorOn(false);
         tape->Tick();
         tape->SetMotorOn(true);
      }
      tape->Tick();
   }

   // Reaching here at all (rather than SIGSEGV) is the actual assertion --
   // the crash this test targets kills the whole test binary, not just
   // this one case, so there is nothing more specific to ASSERT on.
   SUCCEED();
}

// Second, distinct bug candidate found auditing the same "start_record_"
// area for a sibling of the fix above: the "Shorten now the following one"
// block right after the insert/extend logic (still inside `if (record_)`,
// but OUTSIDE the same/different-level branch -- it runs on every tick,
// not just on a transition) does:
//   tape_array_[tape_position_+1].length -= this_tick_time_;
// unconditionally, on a uint64_t field, without checking that length is
// actually >= this_tick_time_ first. Recording only ever holds
// this_tick_time_ at a fixed 4 T-states, so for this to underflow, an
// EXISTING entry immediately ahead of the recording position needs to be
// driven below 4 -- which happens for free if the guest holds the write
// line at a constant level for many consecutive ticks while overdubbing
// onto already-loaded (not blank) tape content: `tape_position_` does not
// move during a same-level run, so the same subsequent entry gets -=4'd
// on every single tick until it wraps. This targets a real loaded tape,
// not a blank one -- the "recording onto an already-populated tape did
// not crash" note from the original bug report was from a short test,
// not proof this path is safe.
TEST(TapeRecording, OverdubOntoLoadedTapeDoesNotUnderflowNextEntryLength)
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

   // Real commercial tape dump already used by Test_Dumps_tape.cpp -- real
   // pulse-length data, not synthetic, so the entries ahead of position 0
   // have genuine, varied lengths to overdub onto.
   machine.LoadTape("./res/Tape/Lemmings (UK) (1991) (01. Level 01 FUN - JUST DIG!) (Version Split) [Original] [TAPE].cdt");
   for (int i = 0; i < 100; ++i)
      machine.RunTimeSlice();

   CTape* tape = machine.GetTape();
   PPI8255* ppi = machine.GetPPI();
   ASSERT_NE(nullptr, tape);
   ASSERT_NE(nullptr, ppi);
   ASSERT_GT(tape->GetNbInversions(), 10u) << "tape did not actually load, test would prove nothing";

   tape->Rewind();
   tape->SetMotorOn(true);
   tape->Record();

   // SetMotorOn() doesn't take effect immediately either (it arms a
   // MOTOR_DELAY countdown), so neither the motor nor start_record_'s
   // transition are guaranteed to have happened after just one tick.
   // Tick until recording is actually live before trusting
   // GetTapePosition() -- otherwise "the entry the shorten logic targets"
   // gets computed from a stale, pre-transition position (this was a real
   // mistake in an earlier version of this test: it watched the position
   // BEFORE start_record_'s own increment, which is index tape_position_
   // itself post-transition -- the entry actively being recorded into,
   // which is SUPPOSED to grow while a level is held constant -- not the
   // shorten target one further ahead).
   ppi->tape_write_data_level_ = false;
   int settle_ticks = 0;
   while (!tape->IsRecordOn() && settle_ticks < 10000)
   {
      tape->Tick();
      ++settle_ticks;
   }
   ASSERT_TRUE(tape->IsRecordOn()) << "recording never actually started";

   unsigned int watch_idx = tape->GetTapePosition() + 1;
   CTape::DebugFlux before;
   ASSERT_TRUE(tape->GetFlux(watch_idx, before));

   // Hold the write line at a constant level: no more transitions, so
   // tape_position_ stays put and the "shorten the following one" block
   // hits the SAME next entry on every remaining tick. The bug does not
   // necessarily crash the process -- it corrupts a length field into a
   // huge value that only blows up later (export, or a subsequent
   // playback pass) -- so check the actual value, don't just hope for a
   // SIGSEGV.
   for (int i = 0; i < 199999; ++i)
      tape->Tick();

   CTape::DebugFlux after;
   ASSERT_TRUE(tape->GetFlux(watch_idx, after));
   // A real (non-wrapped) shortening only ever decreases length towards
   // zero. Wrapping past zero on a uint64_t makes it enormous instead.
   EXPECT_LT(after.length, before.length + 1)
      << "watched entry length grew from " << before.length << " to "
      << after.length << " -- classic uint64_t underflow signature";
}

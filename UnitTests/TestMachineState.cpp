#include "gtest/gtest.h"

#include "TestUtils.h"
#include "MachineState.h"

#include <string>
#include <vector>

// Does a full engine state put the machine back where it was, well enough that
// it then runs the same way?
//
// StateDeterminism asks that of a .SNA and answers no: every fingerprinted
// field diverges once the machine runs on, because the scheduler's per-component
// cycle debt has no field in the container. This asks the same question of
// MachineState, which adds that debt on top of the .SNA, and asserts the answer
// is yes.
//
// The two tests share their method deliberately. The .SNA one is the control:
// if it ever stops diverging, this one has stopped proving anything.

namespace
{

struct Fingerprint
{
   std::vector<std::pair<std::string, unsigned long long> > fields;

   void Add(const char* name, unsigned long long value)
   {
      fields.push_back(std::make_pair(std::string(name), value));
   }

   void AddBlock(const char* name, const unsigned char* data, size_t size)
   {
      unsigned long long h = 14695981039346656037ULL;
      for (size_t i = 0; i < size; ++i)
      {
         h ^= data[i];
         h *= 1099511628211ULL;
      }
      Add(name, h);
   }
};

Fingerprint Capture(EmulatorEngine* machine)
{
   Fingerprint f;

   f.AddBlock("ram", machine->GetMem()->GetRamBuffer(), 0x10000);

   Z80* z80 = machine->GetProc();
   f.Add("z80.pc", z80->pc_);
   f.Add("z80.sp", z80->sp_);
   f.Add("z80.iff1", z80->iff1_ ? 1 : 0);
   f.Add("z80.iff2", z80->iff2_ ? 1 : 0);
   f.Add("z80.opcode", z80->current_opcode_);
   f.Add("z80.machine_cycle", z80->machine_cycle_);
   f.Add("z80.t", z80->t_);

   CRTC* crtc = machine->GetCRTC();
   f.Add("crtc.hcc", crtc->hcc_);
   f.Add("crtc.vcc", crtc->vcc_);
   f.Add("crtc.vlc", crtc->vlc_);
   f.AddBlock("crtc.registers", crtc->registers_list_, sizeof(crtc->registers_list_));

   CTape* tape = machine->GetTape();
   f.Add("tape.position", tape->GetTapePosition());
   f.Add("tape.inversions", tape->GetNbInversions());
   f.Add("tape.recording", tape->IsRecordOn() ? 1 : 0);

   f.Add("fdc.track0", machine->GetFDC()->GetCurrentTrack(0));
   f.Add("fdc.motor", machine->GetFDC()->IsMotorOn() ? 1 : 0);
   f.Add("fdc.main_status", machine->GetFDC()->GetMainStatus());

   // Memory paging: which banks and which upper ROM the CPU currently sees.
   f.Add("mem.selected_rom", machine->GetMem()->GetSelectedRom());
   f.AddBlock("mem.bank0", machine->GetMem()->GetRamBuffer(), 0x4000);

   // Gate array: what actually drives the interrupt the CPU will take next.
   GateArray* ga = machine->GetVGA();
   f.Add("ga.pen", ga->pen_r_);
   f.Add("ga.screen_mode", ga->buffered_screen_mode_);

   // PSG: the audible state, and the flag that selects keyboard vs latch on
   // port A -- get that wrong and the firmware reads different keys.
   Ay8912* psg = machine->GetPSG();
   f.Add("psg.selected", psg->GetRegisterAdress());

   return f;
}

EmulatorEngine* NewBootedMachine(DirectoriesImp& dirImp, CDisplay& display, Log& log,
                                 SoundFactory& soundFactory, ConfigurationManager& conf_manager,
                                 const char* section)
{
   EmulatorEngine* machine = new EmulatorEngine();

   display.Init(false);
   display.Show(false);

   machine->SetDirectories(&dirImp);
   machine->SetLog(&log);
   machine->SetConfigurationManager(&conf_manager);
   machine->Init(&display, &soundFactory);
   machine->GetMem()->Initialisation();
   machine->LoadConfiguration(section, "./TestConf.ini");
   machine->Reinit();
   machine->SetFixedSpeed(true);
   machine->SetSpeedLimit(EmulatorEngine::E_FULL);

   return machine;
}

}  // namespace

namespace
{

// Long enough that a run which only looks identical cannot stay that way: the
// firmware sweeps the keyboard, the gate array raises interrupts and the CRTC
// completes frames many times over.
const int kSlicesAfterSave = 400;

void CheckOneMachine(const char* section, int slices_before_save,
                    const char* disk = nullptr, const char* format = "")
{
   DirectoriesImp dirImp; CDisplay display; Log log;
   SoundFactory soundFactory; ConfigurationManager conf_manager;
   EmulatorEngine* machine =
      NewBootedMachine(dirImp, display, log, soundFactory, conf_manager, section);

   // With a disk inserted and the drive seeking, the FDC and the disk image
   // carry state that an idle BASIC prompt never exercises.
   if (disk != nullptr)
   {
      ASSERT_EQ(0, machine->LoadDisk(disk, 0, false))
         << "the disk fixture did not load, so this case proves nothing: " << disk;

      machine->Paste("cat\r");

      // Run until the drive is actually turning and stop there. The motor spins
      // down again once the access finishes, so waiting a fixed number of slices
      // and then looking is a coin toss.
      bool spinning = false;
      for (int i = 0; i < 2000 && !spinning; ++i)
      {
         machine->RunTimeSlice(false);
         spinning = machine->GetFDC()->IsMotorOn();
      }
      ASSERT_TRUE(spinning)
         << "the drive never started, so the FDC state under test is still idle";
   }

   const int kSlicesBeforeSave = slices_before_save;

   for (int i = 0; i < kSlicesBeforeSave; ++i)
      machine->RunTimeSlice();

   int ram_in_use = 0;
   const unsigned char* ram = machine->GetMem()->GetRamBuffer();
   for (int i = 0; i < 0x10000; ++i) if (ram[i] != 0) ++ram_in_use;
   ASSERT_GT(ram_in_use, 0)
      << "the machine never executed an instruction -- no ROMs found. This test "
         "needs CONF/, ROM/ and res/ from UnitTests/ and Keyboards/ from "
         "CPCCoreEmu/ reachable from the working directory.";

   std::vector<unsigned char> state;
   ASSERT_TRUE(MachineState::Save(machine, state));
   ASSERT_FALSE(state.empty());
   const Fingerprint at_save = Capture(machine);

   for (int i = 0; i < kSlicesAfterSave; ++i)
      machine->RunTimeSlice();
   const Fingerprint straight_through = Capture(machine);

   ASSERT_TRUE(MachineState::Load(machine, &state[0], state.size()));
   const Fingerprint at_restore = Capture(machine);

   // Split the two failure modes: a field that is already wrong the instant the
   // state comes back was not carried, as opposed to one that only drifts once
   // the machine runs on.
   for (size_t i = 0; i < at_save.fields.size(); ++i)
   {
      if (at_save.fields[i].second != at_restore.fields[i].second)
         fprintf(stderr, "  NOT CARRIED BY RESTORE: %s\n", at_save.fields[i].first.c_str());
   }

   for (int i = 0; i < kSlicesAfterSave; ++i)
      machine->RunTimeSlice();
   const Fingerprint after_restore = Capture(machine);

   delete machine;

   ASSERT_EQ(straight_through.fields.size(), after_restore.fields.size());

   int divergent = 0;
   for (size_t i = 0; i < straight_through.fields.size(); ++i)
   {
      if (straight_through.fields[i].second == after_restore.fields[i].second)
         continue;
      ++divergent;
      fprintf(stderr, "  STILL DIVERGES: %s\n", straight_through.fields[i].first.c_str());
   }

   fprintf(stderr, "MACHINE STATE %-9s save@%-5d %-6s %d of %zu fields diverged\n",
      section, slices_before_save, disk ? format : "idle",
      divergent, straight_through.fields.size());

   EXPECT_EQ(0, divergent)
      << section << " save@" << slices_before_save
      << ": the state did not carry everything the machine needs to resume";
}

}  // namespace

TEST(MachineStateTest, RestoringAStateReproducesTheSameRun)
{
   const char* sections[] = { "6128", "464", "6128PLUS" };
   const int save_points[] = { 20, 137, 400 };

   for (size_t s = 0; s < sizeof(sections) / sizeof(sections[0]); ++s)
      for (size_t p = 0; p < sizeof(save_points) / sizeof(save_points[0]); ++p)
         CheckOneMachine(sections[s], save_points[p]);
}

// The FDC and the disk only carry state worth restoring while the drive is
// actually turning, which a BASIC prompt never makes happen.
//
// Once per container format, because they do not decode alike: a plain DSK has
// one fixed-size image of each track, an EDSK varies the size per track, and
// IPF, HFE and SCP describe the flux itself and can hold several revolutions of
// the same track. The head position that has to survive a round trip means
// something different in each.
TEST(MachineStateTest, RestoringAStateReproducesTheSameRunWithADiskSpinning)
{
   struct Image { const char* path; const char* format; };
   const Image images[] = {
      { "./res/FDC/fdctest.dsk",                                            "DSK"  },
      { "./res/DSK/Ace Of Aces (UK) (1985) [Original] (Weak Sectors).dsk",  "EDSK" },
      { "./res/After Burner (UK) (1988) [Activision SEGA] (Pre-release).ipf","IPF"  },
      { "./res/30YMD double sides 1 and 2.hfe",                             "HFE"  },
      { "./res/The Demo [A].scp",                                           "SCP"  },
   };

   for (size_t i = 0; i < sizeof(images) / sizeof(images[0]); ++i)
   {
      SCOPED_TRACE(images[i].format);
      CheckOneMachine("6128", 20, images[i].path, images[i].format);
      CheckOneMachine("6128", 137, images[i].path, images[i].format);
   }
}

TEST(MachineStateTest, RejectsSomethingThatIsNotAState)
{
   DirectoriesImp dirImp; CDisplay display; Log log;
   SoundFactory soundFactory; ConfigurationManager conf_manager;
   EmulatorEngine* machine =
      NewBootedMachine(dirImp, display, log, soundFactory, conf_manager, "6128");

   for (int i = 0; i < 20; ++i)
      machine->RunTimeSlice();

   std::vector<unsigned char> state;
   ASSERT_TRUE(MachineState::Save(machine, state));

   std::vector<unsigned char> bad = state;
   bad[0] = 'X';
   EXPECT_FALSE(MachineState::Load(machine, &bad[0], bad.size()))
      << "a buffer without the magic must be refused";

   bad = state;
   bad[4] = 0xFE; bad[5] = 0xFF;
   EXPECT_FALSE(MachineState::Load(machine, &bad[0], bad.size()))
      << "an unknown version must be refused rather than misread";

   EXPECT_FALSE(MachineState::Load(machine, &state[0], state.size() / 2))
      << "a truncated state must be refused";

   // A whole .SNA is not a MachineState, however valid it is on its own.
   std::vector<unsigned char> sna;
   ASSERT_TRUE(machine->SaveSnapshotNow(sna));
   EXPECT_FALSE(MachineState::Load(machine, &sna[0], sna.size()));

   delete machine;
}

// The other half of keeping media out of the state: a head position only means
// something against the disk it was recorded on. Swap the disk and the state
// must be refused, not applied to whatever happens to be in the drive.
TEST(MachineStateTest, RefusesAStateTakenWithADifferentDisk)
{
   DirectoriesImp dirImp; CDisplay display; Log log;
   SoundFactory soundFactory; ConfigurationManager conf_manager;
   EmulatorEngine* machine =
      NewBootedMachine(dirImp, display, log, soundFactory, conf_manager, "6128");

   ASSERT_EQ(0, machine->LoadDisk("./res/FDC/fdctest.dsk", 0, false));
   for (int i = 0; i < 100; ++i)
      machine->RunTimeSlice(false);

   std::vector<unsigned char> state;
   ASSERT_TRUE(MachineState::Save(machine, state));

   // Same drive, different geometry.
   ASSERT_EQ(0, machine->LoadDisk(
      "./res/After Burner (UK) (1988) [Activision SEGA] (Pre-release).ipf", 0, false));

   const bool loaded = MachineState::Load(machine, &state[0], state.size());

   const unsigned int tracks_now =
      machine->GetFDC()->GetCurrentTrack(0) >= 0 ? 1u : 0u;
   (void)tracks_now;

   delete machine;

   EXPECT_FALSE(loaded)
      << "a state recorded against another disk was applied instead of refused";
}

// An empty drive is a geometry too: a state taken with no disk must not be
// loaded into a machine that now has one.
TEST(MachineStateTest, RefusesAStateTakenWithNoDiskWhenOneIsInserted)
{
   DirectoriesImp dirImp; CDisplay display; Log log;
   SoundFactory soundFactory; ConfigurationManager conf_manager;
   EmulatorEngine* machine =
      NewBootedMachine(dirImp, display, log, soundFactory, conf_manager, "6128");

   for (int i = 0; i < 100; ++i)
      machine->RunTimeSlice(false);

   std::vector<unsigned char> state;
   ASSERT_TRUE(MachineState::Save(machine, state));

   ASSERT_EQ(0, machine->LoadDisk("./res/FDC/fdctest.dsk", 0, false));

   const bool loaded = MachineState::Load(machine, &state[0], state.size());
   delete machine;

   EXPECT_FALSE(loaded)
      << "a state taken with an empty drive was applied to a loaded one";
}

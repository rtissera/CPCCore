#include "gtest/gtest.h"

#include "TestUtils.h"
#include "MachineState.h"

#include <cstdio>
#include <string>
#include <vector>

// What a state actually costs.
//
// A libretro frontend allocates one buffer from retro_serialize_size() and
// reuses it for every save, and rewind keeps dozens of them, so the number
// matters as much as the contents.

namespace
{

EmulatorEngine* Boot(DirectoriesImp& dirImp, CDisplay& display, Log& log,
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
   for (int i = 0; i < 200; ++i)
      machine->RunTimeSlice();
   return machine;
}

unsigned int U32At(const std::vector<unsigned char>& b, size_t at)
{
   return b[at] | (b[at+1] << 8) | (b[at+2] << 16) | ((unsigned int)b[at+3] << 24);
}

}  // namespace

TEST(MachineStateSize, ReportsWhatAStateCosts)
{
   const char* sections[] = { "464", "6128", "6128PLUS" };

   for (size_t s = 0; s < sizeof(sections) / sizeof(sections[0]); ++s)
   {
      DirectoriesImp dirImp; CDisplay display; Log log;
      SoundFactory soundFactory; ConfigurationManager conf_manager;
      EmulatorEngine* machine = Boot(dirImp, display, log, soundFactory, conf_manager, sections[s]);

      std::vector<unsigned char> state;
      ASSERT_TRUE(MachineState::Save(machine, state));

      std::vector<unsigned char> sna;
      ASSERT_TRUE(machine->SaveSnapshotNow(sna));

      delete machine;

      const unsigned int sna_len = U32At(state, 8);
      fprintf(stderr, "\nSTATE SIZE %-9s total %8zu bytes (%6.1f KB)\n",
         sections[s], state.size(), state.size() / 1024.0);
      fprintf(stderr, "  .SNA image        %8u bytes (%6.1f KB)  %5.1f%%\n",
         sna_len, sna_len / 1024.0, 100.0 * sna_len / state.size());
      fprintf(stderr, "  header              %8d bytes\n", 12);

      // Walk the chunks that follow the .SNA.
      size_t at = 12 + sna_len;
      size_t chunk_total = 0;
      while (at + 8 <= state.size())
      {
         const unsigned int id = U32At(state, at);
         const unsigned int len = U32At(state, at + 4);
         char name[5] = {0};
         name[0] = (char)((id >> 24) & 0xFF); name[1] = (char)((id >> 16) & 0xFF);
         name[2] = (char)((id >> 8) & 0xFF);  name[3] = (char)(id & 0xFF);
         fprintf(stderr, "  chunk %-4s          %8u bytes\n", name, len + 8);
         chunk_total += len + 8;
         at += 8 + len;
      }
      fprintf(stderr, "  chunks total      %8zu bytes  %5.1f%%\n",
         chunk_total, 100.0 * chunk_total / state.size());
      fprintf(stderr, "  vs a plain .SNA   %8zu bytes  (+%.1f%%)\n",
         state.size() - sna.size(),
         100.0 * (state.size() - sna.size()) / sna.size());

      EXPECT_EQ(state.size(), 12 + sna_len + chunk_total)
         << "the chunk walk did not account for the whole state";
   }
}

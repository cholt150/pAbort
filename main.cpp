// main file for handling raw tcp packets, especially an abort message
// created by matt waltz
// email matthewwaltzis@gmail.com with any questions

#include "pAbortModule.h"
#include "MOOS/libMOOS/App/MOOSApp.h"

// here we do some command line parsing ...
// mission file could be first free parameter
// app name can be the second free parameter

int main(int argc, char *argv[])  {
  MOOS::CommandLineParser P(argc, argv);

  std::string mission_file = P.GetFreeParameter(0, "mission.moos");
  std::string app_name = P.GetFreeParameter(1, "pAbort");

  pAbortModule App;
  App.Run(app_name, mission_file, argc, argv);

  return 0;
}

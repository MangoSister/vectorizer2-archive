#define CATCH_CONFIG_RUNNER
#include <Catch2/single_include/catch2/catch.hpp>

int main(int argc, char *argv[])
{
    Catch::Session session; // There must be exactly one instance

    std::string outputBaseDirectoryStr;

    // Build a new parser on top of Catch's
    using namespace Catch::clara;
    auto cli
        = session.cli() // Get Catch's composite command line parser
        | Opt(outputBaseDirectoryStr, "Output Base Directory") // bind variable to a new option, with a hint string
        ["-o"]["--output"]    // the option names it will respond to
        ("Output Base Directory");        // description string for the help output

    // Now pass the new composite back to Catch so it uses that
    session.cli(cli);

    // Let Catch (using Clara) parse the command line
    int returnCode = session.applyCommandLine(argc, argv);
    if (returnCode != 0) // Indicates a command line error
        return returnCode;

    return session.run();
}
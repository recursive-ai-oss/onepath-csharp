// Runs the runner headless under Node: `node main.mjs [runner options]` from the AppBundle folder.
import { dotnet } from './_framework/dotnet.js';

const { runMain } = await dotnet.withApplicationArguments(...process.argv.slice(2)).create();
const exitCode = await runMain();
process.exit(exitCode);

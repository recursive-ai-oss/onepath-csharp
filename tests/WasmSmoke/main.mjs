// Runs the smoke app headless under Node: `node main.mjs` from the published/built AppBundle folder.
import { dotnet } from './_framework/dotnet.js';

const { runMain } = await dotnet.create();
const exitCode = await runMain();
process.exit(exitCode);

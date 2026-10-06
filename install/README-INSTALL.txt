================================================================================
Nova Calibrator - Installation
================================================================================

IMPORTANT: Close Steam completely before installing (not just SteamVR).
Steam.exe itself locks the driver files. Check the system tray and exit Steam
fully.

There are two ways to install: the script (easiest) or manually.


--------------------------------------------------------------------------------
OPTION 1: Script install (recommended)
--------------------------------------------------------------------------------

1. Extract this entire zip to a folder, e.g. Downloads\Nova Calibrator.
   Keep the folder structure intact - Install.ps1 reads the "app" and "driver"
   folders that sit next to it.

2. Unblock the extracted files (Windows blocks scripts downloaded from the
   internet):
     - Select ALL files in the extracted folder, right-click -> Properties,
       tick "Unblock" at the bottom, OK.
     - Or open PowerShell in the folder and run:
         Get-ChildItem -Recurse | Unblock-File

3. Right-click Install.ps1 -> "Run with PowerShell".

4. Click "Yes" when Windows asks for administrator permission.

5. Answer the optional module question. The Lighthouse module adds base
   station tools, starting with a tab showing each base station and which
   lighthouse-tracked devices can see it; later versions add more. To add
   or remove it later, run Install.ps1 again.

6. Done. Start SteamVR - Nova Calibrator appears as a dashboard overlay and
   starts automatically with SteamVR.

Nova Calibrator was called QuestCalibrator before 1.2.0. If QuestCalibrator is
installed, Install.ps1 removes it first and keeps your calibration and
settings: Nova Calibrator brings them over the first time it starts.

To uninstall: run Uninstall.ps1 the same way, or find "Nova Calibrator" in
Windows Settings -> Apps.


--------------------------------------------------------------------------------
OPTION 2: Manual install (no scripts)
--------------------------------------------------------------------------------

You need to know where SteamVR is installed. The default is:

    C:\Program Files (x86)\Steam\steamapps\common\SteamVR

If Steam is on another drive, look for
    <drive>\SteamLibrary\steamapps\common\SteamVR
If you use a custom Steam library folder, SteamVR is under
    <that folder>\steamapps\common\SteamVR

Steps:

1. Close Steam completely (not just SteamVR).

2. Create a permanent home for the app, e.g.
       C:\Program Files\NovaCalibrator
   and copy the contents of this package's "app" folder into it
   (NovaCalibrator.exe, openvr_api.dll, manifest.vrmanifest, icon.png).
   Don't run it straight out of Downloads - the app registers its own path
   with SteamVR, so it needs a stable location.

3. Remove conflicting drivers, if present. Inside the SteamVR folder, delete
   these folders if they exist:
       drivers\01spacecalibrator
       drivers\000spacecalibrator
       drivers\01questcalibrator
   (OpenVR-SpaceCalibrator conflicts with Nova Calibrator, and so does
   QuestCalibrator, its earlier name; two of them installed at once will
   double-apply offsets. Your calibration and settings carry over by
   themselves.)

4. Copy this package's "driver\01novacalibrator" folder into the SteamVR
   "drivers" folder, so you end up with:
       <SteamVR>\drivers\01novacalibrator\driver.vrdrivermanifest
       <SteamVR>\drivers\01novacalibrator\bin\win64\driver_01novacalibrator.dll
       <SteamVR>\drivers\01novacalibrator\resources\...

5. Register the overlay with SteamVR (required for auto-start and the
   dashboard listing). Open a Command Prompt in the folder from step 2 and
   run:
       NovaCalibrator.exe -installmanifest

   A message box reports the result: either the manifest path that was
   registered, or the reason it failed. Nova Calibrator is a windowed
   application, so the Command Prompt returns immediately and any text it
   prints there is unreliable - the dialog is the authoritative result.

   If you skip this step the driver still works, but you must launch
   NovaCalibrator.exe by hand every session.

6. Optional: enable the Lighthouse module. In an administrator Command
   Prompt, run:
       reg add HKLM\Software\NovaCalibrator\Modules /v Lighthouse /t REG_DWORD /d 1 /f

7. Start SteamVR.

Manual uninstall:

1. Close SteamVR.
2. In a Command Prompt in the app folder, run:
       NovaCalibrator.exe -removemanifest
   Do this BEFORE deleting anything: the command finds the registration to
   remove using the manifest file sitting next to the executable.
3. Delete <SteamVR>\drivers\01novacalibrator
4. Delete the app folder from step 2.
5. If you enabled the Lighthouse module, run (as administrator):
       reg delete HKLM\Software\NovaCalibrator /f


--------------------------------------------------------------------------------
Notes
--------------------------------------------------------------------------------

- Windows SmartScreen may warn about NovaCalibrator.exe because it isn't
  code-signed. Click "More info" -> "Run anyway". This is expected.
- Nova Calibrator requires SteamVR's "multiple drivers" support. The app turns
  that setting on every time it starts, so both install methods are covered -
  the script install simply does it up front. Uninstalling deliberately leaves
  the setting enabled, because other OpenVR tools depend on it too.
- If SteamVR has never been run on this PC, start it once and close it before
  installing. Both the installer and -installmanifest ask the OpenVR runtime
  where SteamVR lives, and that information doesn't exist until SteamVR has
  run at least once.
================================================================================

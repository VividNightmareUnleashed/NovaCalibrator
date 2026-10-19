================================================================================
Nova Calibrator - Installation
================================================================================

CAUTION: Close Steam completely before you install or remove Nova Calibrator.
Steam locks the driver files. If you close only SteamVR, the installation can
fail.

NOTE: If SteamVR has not started on this PC before, start SteamVR one time.
Then close Steam. The installer gets the SteamVR folder from the OpenVR
runtime. The OpenVR runtime has this data only after SteamVR starts one time.

You can install Nova Calibrator in two ways: with the installer script, or
manually without scripts. The installer script is the easier procedure.


--------------------------------------------------------------------------------
OPTION 1: Install with the script (recommended)
--------------------------------------------------------------------------------

1.  Close Steam.
2.  Make sure that the Steam icon is not in the system tray.
3.  Extract all the files in this zip file to a folder, for example
    Downloads\Nova Calibrator.
4.  Do not move files out of the "app" and "driver" folders.

    NOTE: Install.ps1 reads the "app" and "driver" folders next to it.

5.  Unblock the extracted files. Windows blocks scripts that come from the
    internet. Do one of these procedures:

    a.  Select all the extracted files.
    b.  Right-click the files and select "Properties".
    c.  Select the "Unblock" check box.
    d.  Click "OK".

    or:

    a.  Open PowerShell in the extracted folder.
    b.  Type this command:

            Get-ChildItem -Recurse | Unblock-File

6.  Right-click Install.ps1 and select "Run with PowerShell".
7.  When Windows asks for administrator permission, click "Yes".
8.  When the installer shows "Install the Lighthouse module? [Y/N]", type Y
    or N.

    NOTE: The Lighthouse module adds a page that shows each base station and
    the Lighthouse devices that can see it. Later versions add more base
    station tools. To add or remove the module later, do this procedure
    again.

9.  If the installer shows "Uninstall it and continue? [Y/N]", type Y.

    NOTE: The installer shows this question if OpenVR-SpaceCalibrator is
    installed. If two calibrators operate together, they apply the
    calibration two times.

10. Start SteamVR.

Nova Calibrator starts with SteamVR and shows in the SteamVR dashboard.

The name of Nova Calibrator was QuestCalibrator before version 1.2.0. If
QuestCalibrator is installed, Install.ps1 removes it first. Nova Calibrator
keeps your calibration and settings. It copies them the first time that it
starts.

To remove Nova Calibrator:

1.  Close Steam.
2.  Make sure that the Steam icon is not in the system tray.
3.  Right-click Uninstall.ps1 in the extracted folder and select
    "Run with PowerShell".
4.  When Windows asks for administrator permission, click "Yes".

NOTE: You can also remove "Nova Calibrator" in Windows "Settings" > "Apps".


--------------------------------------------------------------------------------
OPTION 2: Install manually (no scripts)
--------------------------------------------------------------------------------

For this procedure, you must know the SteamVR folder. The default SteamVR
folder is:

    C:\Program Files (x86)\Steam\steamapps\common\SteamVR

If Steam is on a different drive, the SteamVR folder is usually:

    <drive>\SteamLibrary\steamapps\common\SteamVR

If you use a custom Steam library folder, the SteamVR folder is:

    <that folder>\steamapps\common\SteamVR

CAUTION: Do not start NovaCalibrator.exe from the Downloads folder. The app
records its folder in SteamVR. If you move or delete that folder, SteamVR
cannot start the app.

1.  Close Steam.
2.  Make sure that the Steam icon is not in the system tray.
3.  Make a permanent folder for the app, for example
    C:\Program Files\NovaCalibrator.
4.  Copy the contents of the "app" folder in this package to the new folder.

    NOTE: The "app" folder contains NovaCalibrator.exe, openvr_api.dll,
    manifest.vrmanifest and icon.png.

5.  In the SteamVR folder, delete these folders if they are there:

        drivers\01spacecalibrator
        drivers\000spacecalibrator
        drivers\01questcalibrator

    NOTE: These folders contain the drivers of OpenVR-SpaceCalibrator and
    QuestCalibrator. QuestCalibrator is the earlier name of Nova Calibrator.
    If two calibrators operate together, they apply the calibration two
    times. Nova Calibrator keeps your QuestCalibrator calibration and
    settings.

6.  Copy the "driver\01novacalibrator" folder in this package to the
    "drivers" folder in the SteamVR folder.
7.  Make sure that these files are in the SteamVR folder:

        <SteamVR>\drivers\01novacalibrator\driver.vrdrivermanifest
        <SteamVR>\drivers\01novacalibrator\bin\win64\driver_01novacalibrator.dll
        <SteamVR>\drivers\01novacalibrator\resources\...

8.  Open a Command Prompt in the app folder from step 3.
9.  Type this command:

        NovaCalibrator.exe -installmanifest

    NOTE: This command registers Nova Calibrator with SteamVR. Then SteamVR
    starts Nova Calibrator automatically and shows it in the dashboard.
    A message box shows the result. The result is the registered manifest
    path or the cause of the failure. Nova Calibrator is a windowed app.
    Thus, the Command Prompt does not wait for it, and text in the Command
    Prompt is not reliable. The message box shows the correct result.

    If you do not do steps 8 and 9, the driver operates. But you must start
    NovaCalibrator.exe manually each time that you use SteamVR.

10. Optional: To add the Lighthouse module, do these steps:

    a.  Open a Command Prompt as administrator.
    b.  Type this command:

            reg add HKLM\Software\NovaCalibrator\Modules /v Lighthouse /t REG_DWORD /d 1 /f

11. Start SteamVR.

To remove a manual installation:

CAUTION: Do steps 3 and 4 before you delete files. The command in step 4
finds the SteamVR registration through the manifest file next to
NovaCalibrator.exe.

1.  Close Steam.
2.  Make sure that the Steam icon is not in the system tray.
3.  Open a Command Prompt in the app folder.
4.  Type this command:

        NovaCalibrator.exe -removemanifest

5.  Delete the <SteamVR>\drivers\01novacalibrator folder.
6.  Delete the app folder.
7.  If you added the Lighthouse module, do these steps:

    a.  Open a Command Prompt as administrator.
    b.  Type this command:

            reg delete HKLM\Software\NovaCalibrator /f


--------------------------------------------------------------------------------
Notes
--------------------------------------------------------------------------------

- NovaCalibrator.exe does not have a code signature. Thus, Windows
  SmartScreen can show a warning for it. This is normal. To start the app,
  click "More info". Then click "Run anyway".
- Nova Calibrator needs the SteamVR option for multiple drivers. The app sets
  this option to on each time that it starts. Thus, the two installation
  procedures give the same result. The installer script also sets the option
  before the first start. Removal does not set the option to off, because
  other OpenVR tools also need it.
================================================================================

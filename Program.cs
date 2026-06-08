//Created on Tue Apr 21 11:59:15 2026
//Author: Marcelo ROLOTTI, BOHM Lab IPNP
//
using System;
using System.Collections.Generic;
//using System.Linq; //In case more devices need to be managed
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Thorlabs.MotionControl.DeviceManagerCLI;
using Thorlabs.MotionControl.GenericMotorCLI;
//using Thorlabs.MotionControl.GenericMotorCLI.ControlParameters; //In case we want to change velocity
using Thorlabs.MotionControl.GenericMotorCLI.Settings; //Sometimes needed for MotorConfiguration
using Thorlabs.MotionControl.VerticalStageCLI;

namespace KVS30StageStepper
{
    enum ExperimentState
    {
        Initialization,
        Alignment,
        ParameterInput,
        Running,
        PostExperiment,
        Exit
    }
    class Program
    {
        const decimal MaxLimit = 30.0m; //Global physical constraints of stage (KVS30)
        const decimal MinLimit = 0.0m; //Change these global constraints for other stages
        static bool _keepRunning = true; //Safety bool to prevent unsafe exit
        static bool _softStop = false;
        static ExperimentState _currentState = ExperimentState.Initialization;
        static VerticalStage _device;
        static StepwiseParameters _expParams = new StepwiseParameters();
        static DateTime _startTime;
        #region Main
        static void Main()
        {
            Console.OutputEncoding = System.Text.Encoding.UTF8;
            Console.CancelKeyPress += (sender, e) =>
            {
                e.Cancel = true; //prevent unsafe exit
                _keepRunning = false;
                Console.WriteLine("\n[ABORT] Safely shutting down...");
            };

            try
            {
                while (_keepRunning && _currentState != ExperimentState.Exit)
                {
                    if ((_currentState == ExperimentState.ParameterInput || _currentState == ExperimentState.Running) && _device.Position != _expParams.StartPos)
                    {
                        Console.WriteLine("Moving to experiment start position...");
                        decimal returnDist = Math.Abs(_device.Position - _expParams.StartPos);
                        MotorDirection dir = _expParams.StepSize >= 0 ? MotorDirection.Backward : MotorDirection.Forward;
                        PerformSafeMove(dir, returnDist);
                    }
                    switch (_currentState)
                    {
                        case ExperimentState.Initialization:
                            RunInitialization();
                            break;
                        case ExperimentState.Alignment:
                            RunAlignmentMode();
                            break;
                        case ExperimentState.ParameterInput:
                            RunUserConfiguration();
                            break;
                        case ExperimentState.Running:
                            RunExperiment();
                            break;
                        case ExperimentState.PostExperiment:
                            RunPostExperiment();
                            break;
                    }
                }
            }

            /*
            try
            {
                //2. Get Alignment
                decimal alignmentPos = RunAlignmentMode(_device);
                if (alignmentPos == -1.0m) { return; } //safety check

                //3. Get Parameters
                StepwiseParameters config = RunUserConfiguration(alignmentPos);
                if (config == null) { return;} //safety check (user abort)

                //4. Hardware Execution
                Console.WriteLine("Press any key to initialize hardware...");
                //ReadKey is synchronous, we need to allow for abort
                while (!Console.KeyAvailable)
                {
                    if (!_keepRunning) return;
                    Thread.Sleep(50);
                }
                Console.ReadKey(true);
                RunExperiment(_device, config);
            } */
            catch (Exception ex) //safety check
            {
                Console.WriteLine($"\n[System ERROR]: {ex.Message}");
            }
            finally //safely disconnect from hardware
            {
                if (_device != null && _device.IsConnected)
                {
                    Console.WriteLine("Cleaning up hardware connection...");
                    _device.StopImmediate(); //stop motor
                    _device.DisableDevice(); //turn off motor
                    _device.StopPolling(); //stop data stream
                    _device.Disconnect(true); //disconnect
                    _device.Dispose(); //cleanup
                }
                Console.WriteLine("Hardware disconnected safely.\nPress any key to exit.");
                Console.ReadKey();
            }
        }
        #endregion
        #region Initialization
        static void RunInitialization()
        {
            Console.WriteLine("- - - KVS30 Stepwise Controller - - -");
            Console.WriteLine("Press CTRL+C at any time to emergency exit the program.");
            DeviceManagerCLI.BuildDeviceList();
            string serialNo = "24479544";
            _device = VerticalStage.CreateVerticalStage(serialNo);

            Console.WriteLine($"Connecting to _device {serialNo}...");
            _device.Connect(serialNo);

            if (!_device.IsSettingsInitialized())
            {
                _device.WaitForSettingsInitialized(10000);
            }

            var motorSettings = _device.LoadMotorConfiguration(serialNo);

            _device.StartPolling(250);
            _device.EnableDevice();
            Thread.Sleep(1000); //Wait for power-up

            //Set a safe velocity
            var velocityParams = _device.GetVelocityParams();
            velocityParams.MaxVelocity = 2.0m;
            _device.SetVelocityParams(velocityParams);

            //Home Stage
            Console.WriteLine("Homing stage... please wait.");
            _device.Home(60000);

            //Safety check -- _device connection
            if (!_device.IsConnected || _device.NeedsHoming)
            {
                Console.WriteLine("\n[CRITICAL ERROR]: Hardware is not ready.");
                Console.WriteLine("Check USB connection and Power Supply.");
                _currentState = ExperimentState.Exit;
                return; // Exit the program immediately
            }
            Console.WriteLine("Hardware Check Passed: Device Homed and Ready.");
            _currentState = ExperimentState.Alignment;
        }
        #endregion
        #region Alignment
        static void RunAlignmentMode()
        {
            Console.WriteLine("\n- - -ALIGNMENT MODE- - -");
            Console.WriteLine("[Q/A]: 0.01mm ↑↓ | [W/S]: 0.1mm ↑↓ | [E/D]: 0.5mm ↑↓ | [R/F]: 2.0mm ↑↓ | [ENTER]: Lock and start experiment");
            Console.WriteLine("-----------------------------------------------------------------------------------------------------------\n");

            bool aligned = false;
            while (!aligned)
            {
                if (!_keepRunning)
                {
                    _device.StopImmediate(); //hardware emergency stop
                    _currentState = ExperimentState.Exit;
                }
                //Display current position
                Console.Write($"\rCurrent Position: {_device.Position:F3} mm             ");

                while (!Console.KeyAvailable)
                {
                    if (!_keepRunning) _currentState = ExperimentState.Exit;
                    Thread.Sleep(50); //Safe polling interval for CPU
                }
                var key = Console.ReadKey(true).Key;

                switch (key)
                {
                    //PerformSafeMove(string currentSlice, VerticalStage _device, MotorDirection dir, decimal dist)
                    case ConsoleKey.R: PerformSafeMove(MotorDirection.Forward, 2.0m); break;
                    case ConsoleKey.F: PerformSafeMove(MotorDirection.Backward, 2.0m); break;
                    case ConsoleKey.E: PerformSafeMove(MotorDirection.Forward, 0.5m); break;
                    case ConsoleKey.D: PerformSafeMove(MotorDirection.Backward, 0.5m); break;
                    case ConsoleKey.W: PerformSafeMove(MotorDirection.Forward, 0.1m); break;
                    case ConsoleKey.S: PerformSafeMove(MotorDirection.Backward, 0.1m); break;
                    case ConsoleKey.Q: PerformSafeMove(MotorDirection.Forward, 0.01m); break;
                    case ConsoleKey.A: PerformSafeMove(MotorDirection.Backward, 0.01m); break;
                    case ConsoleKey.Enter: aligned = true; break;
                    }
                Thread.Sleep(10); //Small sleep to give time
                while (Console.KeyAvailable) { Console.ReadKey(true); } //flush the buffer
            }
            Console.WriteLine($"\nPosition Locked at: {_device.Position:F3} mm");
            _expParams.StartPos = _device.Position;
            _currentState = ExperimentState.ParameterInput;
        }
        #endregion
        #region Parameter Input
        //Function which allows use of abort while awaiting input from ReadLine
        static string SafeReadLine()
        {
            string input = "";
            while (_keepRunning)
            {
                if (Console.KeyAvailable)
                {
                    var key = Console.ReadKey(false);
                    if (key.Key == ConsoleKey.Enter)
                    {
                        Console.WriteLine();
                        return input;
                    }
                    if (key.Key == ConsoleKey.Backspace && input.Length > 0)
                    {
                        input = input.Substring(0, input.Length - 1);
                        Console.Write("\b \b");
                    }
                    else if (!char.IsControl(key.KeyChar))
                    {
                        input += key.KeyChar;
                    }
                }
                Thread.Sleep(50);
            }
            return null;
        }
        //Function which asks user for parameter input
        static void RunUserConfiguration()
        {
            string mode = "";

            //1. Mode Selection
            while (mode != "1" && mode != "2")
            {
                Console.WriteLine($"\nStarting at Aligned Position: {_expParams.StartPos:F3} mm");
                Console.WriteLine("Choose Input Mode:");
                Console.WriteLine("[1] Finish Position and Number of Slices");
                Console.WriteLine("[2] Step Size and Number of Slices");
                mode = SafeReadLine();
                if (mode == null) //check first if user aborted
                {
                    _currentState = ExperimentState.Exit;
                    return; //exit this method and go back to main
                }
                if (mode != "1" && mode != "2") Console.WriteLine("Invalid selection. Please enter 1 or 2.");
            }

            //2. Collect & Validate Positions
            bool isSafe = false; //safe math & valid positions -> isSafe = true
            while (!isSafe)
            {
                if (mode == "1")
                {
                    decimal finishPos;
                    Console.Write("Enter Finishing Position (0-30 mm): ");
                    while (true)
                    {
                        string rawInput = SafeReadLine();
                        if (rawInput == null) { _currentState = ExperimentState.Exit; return; } //first check ctrl+c
                        if (decimal.TryParse(rawInput, out finishPos)) { break; } //try to parse the number
                        Console.Write("Invalid input. Enter a numeric position: ");
                    }
                    
                    Console.Write("Enter Number of Slices: ");
                    while (true) //SafeReadLine() needs to be called this way
                    {
                        string rawInput = SafeReadLine();
                        if(rawInput == null) { _currentState = ExperimentState.Exit; return; }
                        int tempSlices;
                        if (int.TryParse(rawInput, out tempSlices) && tempSlices > 0)
                        {
                            _expParams.TotalSlices = tempSlices;
                            break;
                        }
                        Console.Write("Invalid input. Enter a whole number of slices (1 or more): ");
                    }

                    //Calculate step size
                    if (_expParams.TotalSlices > 1)
                    {
                        _expParams.StepSize = (finishPos - _expParams.StartPos) / (_expParams.TotalSlices - 1);
                        Console.WriteLine($"\nCalculated Step Size: {_expParams.StepSize} mm");
                    }
                    else //single slice
                    {
                        _expParams.StepSize = 0;
                        finishPos = _expParams.StartPos;
                        Console.WriteLine("Single slice: finish position set to current position.");
                    }

                    //Bounds Check
                    _expParams.FinishPos = finishPos;

                    if (_expParams.FinishPos >= 0 && _expParams.FinishPos <= 30)
                        isSafe = true;
                    else
                        Console.WriteLine("Error: Positions must be between 0-30mm.");
                }
                else //Mode 2
                {
                    Console.Write("Enter Step Size in mm (use negative for downward): ");
                    while (true)
                    {
                        string rawInput = SafeReadLine();
                        if (rawInput == null) {  _currentState = ExperimentState.Exit; return; };
                        decimal tempStepSize;
                        if (decimal.TryParse(rawInput, out tempStepSize))
                        {
                            _expParams.StepSize = tempStepSize;
                            break;
                        }
                        Console.Write("Invalid Input. Enter numeric step size.");
                    }

                    Console.Write("Enter Number of Slices: ");
                    while (true)
                    {
                        string rawInput = SafeReadLine();
                        if (rawInput == null) {  _currentState = ExperimentState.Exit; return; };
                        int tempSlices;
                        if (int.TryParse(rawInput, out tempSlices) && tempSlices > 0)
                        {
                            _expParams.TotalSlices = tempSlices;
                            break;
                        }
                        Console.Write("Invalid input. Enter a whole number of slices (1 or more): ");
                    }

                    decimal predictedFinish = _expParams.StartPos + (_expParams.StepSize * (_expParams.TotalSlices - 1));
                    _expParams.FinishPos = predictedFinish;

                    if (predictedFinish >= 0 && predictedFinish <= 30)
                        isSafe = true;
                    else
                        Console.WriteLine($"Error: This move would end at {predictedFinish}mm (Out of Bounds).");
                }
            }

            //3. Collect Interval in seconds
            Console.Write("Enter Time Interval (seconds): ");
            double intervalSec; //temp for s; hardware takes ms
            while (true)
            {
                string rawInput = SafeReadLine();
                if (rawInput == null) {  _currentState = ExperimentState.Exit; return; };
                if (double.TryParse(rawInput, out intervalSec) && intervalSec > 0)
                {
                    if (intervalSec > 43200)
                    {
                        Console.Write("Interval is too long (max 12h).");
                        continue;
                    }
                    _expParams.IntervalMs = (int)(intervalSec * 1000); //convert to ms for hardware
                    break;
                }
                Console.Write("Invalid input. Enter a time in seconds (e.g. 10 or 0.5): ");
            }
            
            //Final check
            Console.WriteLine("\n--- Parameter Summary ---");
            Console.WriteLine($"Start Pos: {_expParams.StartPos:F3} mm");
            Console.WriteLine($"Finish Pos: {_expParams.FinishPos:F3} mm");
            Console.WriteLine($"Step Size: {_expParams.StepSize:F3} mm");
            Console.WriteLine($"Slices: {_expParams.TotalSlices}");
            Console.WriteLine($"Interval: {_expParams.IntervalMs / 1000.0:F1} seconds");
            Console.WriteLine("-------------------------");
            Console.WriteLine("Press ENTER to confirm or Ctrl+C to abort.");
            if (SafeReadLine() == null) {  _currentState = ExperimentState.Exit; return; };
            _currentState = ExperimentState.Running;
        }
        #endregion
        #region Movement Logic
        //.MoveTo and .MoveRelative are synchronous; can't abort until move finished
        //Need method to abort and stop during movement
        static bool PerformSafeMove(MotorDirection dir, decimal dist)
        {
            decimal absDist = Math.Abs(dist);
            decimal tPos = (dir == MotorDirection.Forward) ? _device.Position + absDist : _device.Position - absDist;
            //boundary check
            if (tPos >= MinLimit && tPos <= MaxLimit)
            {
                //Start move on background thread
                var moveTask = Task.Run(() => _device.MoveRelative(dir, absDist, 60000));
                //Monitor task and ctrl+c
                while (!moveTask.IsCompleted)
                {
                    if (!_keepRunning)
                    {
                        _device.StopImmediate(); //kill hardware movement
                        return false; //abort by user
                    }

                    Console.Write($"\r[MOVING] Position : {_device.Position:F3} mm    ");
                    Thread.Sleep(50);
                }
                //Catch mid-move exception
                if (moveTask.IsFaulted)
                {
                    Console.WriteLine($"\n[HARDWARE FAULT]: {moveTask.Exception?.InnerException?.Message}");
                    return false;
                }
                return true; //Move finished successfully
            }
            else //Out of bounds
            {
                Console.WriteLine($"\n[REJECTED] Target {tPos:F3}mm is out of bounds ({MinLimit}mm - {MaxLimit}mm).");
                return false; //Rejected by bound check
            }
        }
        #endregion
        #region RunExperiment
        static void RunExperiment()
        {
            SessionLogger.Initialize(_expParams.StepSize, _expParams.TotalSlices, _expParams.IntervalMs);
            _startTime = DateTime.Now;
            Console.WriteLine($"\nExperiment started at: {_startTime:HH:mm:ss}");
            Console.WriteLine("---------------------------------------------");

            try
            {
                //Logical boundary check
                decimal totalTravel = _expParams.StartPos + (_expParams.StepSize * (_expParams.TotalSlices - 1));
                if (totalTravel < MinLimit || totalTravel > MaxLimit)
                {
                    throw new Exception($"Calculated path ({totalTravel:F3}mm) is out of bounds.");
                }

                decimal currentTarget = _expParams.StartPos;

                for (int i = 0; i < _expParams.TotalSlices; i++)
                {
                    if (!_keepRunning)
                    {
                        _device.StopImmediate(); //hardware emergency stop
                        _currentState = ExperimentState.Exit;
                        return; //break out to finally block
                    }
                    DateTime now = DateTime.Now;
                    DateTime nextMoveTime = now.AddMilliseconds(_expParams.IntervalMs);
                    string currentSlice = $"[Slice {i+1}/{_expParams.TotalSlices}]";
                    Console.WriteLine($"\n{currentSlice}");
                    switch (i) //decide if scan is first, last, or middle
                    {
                        case 0:
                            Console.WriteLine($"Scan started at alignment position {currentTarget:F3}mm at {now:HH:mm:ss}");
                            break;
                        case int last when i == _expParams.TotalSlices - 1:
                            Console.WriteLine($"Final scan started at position {currentTarget:F3}mm at {now:HH:mm:ss}");
                            break;
                        default:
                            Console.WriteLine($"Slice {i + 1} started at position {currentTarget:F3}mm at {now:HH:mm:ss}");
                            break;
                    }

                    Console.WriteLine($"Performing scan until {nextMoveTime:HH:mm:ss}");
                    //wait for interval length, but keep checking _keepRunning for safety
                    while (DateTime.Now < nextMoveTime && _keepRunning)
                    {
                        Thread.Sleep(50); //check _keepRunning every 50ms
                    }
                    if (!_keepRunning)
                    {
                        _device.StopImmediate();
                        _currentState = ExperimentState.Exit;
                        return;
                    }
                    SessionLogger.LogSlice(currentSlice, now, nextMoveTime, _device.Position);

                    if (i < _expParams.TotalSlices - 1)
                    {
                        Console.WriteLine("\nMoving to next position...");
                        MotorDirection Dir = _expParams.StepSize >= 0 ? MotorDirection.Forward : MotorDirection.Backward; //Determine direction
                        bool moveSuccess = PerformSafeMove(Dir, _expParams.StepSize); //Abortable move
                        if (!moveSuccess || !_keepRunning)
                        {
                            _device.StopImmediate();
                            _currentState = ExperimentState.Exit;
                            return;
                        }
                        currentTarget += _expParams.StepSize;
                    }
                }

                //4. Finishing steps
                Console.WriteLine("\nExperiment Complete.");
                Console.Beep(880, 1000);
                _currentState = ExperimentState.PostExperiment;
            }
            catch (Exception ex)
            {
                Console.WriteLine($"Hardware Error: {ex.Message}");
                _device.StopImmediate();
                _currentState = ExperimentState.Exit;
            }
            _currentState = ExperimentState.PostExperiment;
        }
        #endregion
        #region Post Experiment
        static void RunPostExperiment()
        {
            Console.WriteLine("\n==============================");
            Console.WriteLine("       POST-EXPERIMENT        ");
            Console.WriteLine("\n==============================");

            try
            {
                _device.StopImmediate();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Hardware Warning] {ex.Message}");
            }

            SessionLogger.FinalizeLogFile();

            TimeSpan duration = DateTime.Now - _startTime;

            Console.WriteLine("\n--- Experiment Summary ---");
            Console.WriteLine($"Duration            : {duration:hh\\:mm\\:ss}");
            Console.WriteLine($"Total Slices Written: {SessionLogger._slicesLogged}");
            Console.WriteLine($"Final Position      : {_device.Position:F3} mm");
            Console.WriteLine("---                    ---\n");

            Console.WriteLine("\n--- Post-Experiment Menu ---");
            Console.WriteLine("Select next action:");
            Console.WriteLine("[0] Exit application");
            Console.WriteLine("[1] Return to Alignment Mode");
            Console.WriteLine("[2] Return to Parameter Input (Use CURRENT position as new alignment)");
            Console.WriteLine("[3] Return to Parameter Input (Move back to ORIGINAL alignment)");
            Console.WriteLine("[4] Rerun EXACT same experiment (Move backt to start and run)");
            Console.WriteLine("\n Select an option: ");

            ConsoleKey chosenkey = ConsoleKey.NoName;
            while (_keepRunning)
            {
                if (Console.KeyAvailable)
                {
                    chosenkey = Console.ReadKey(true).Key;
                    break;
                }
                Thread.Sleep(20);
            }

            if (!_keepRunning) { _currentState = ExperimentState.Exit; return; }

            switch (chosenkey)
            {
                case ConsoleKey.D0:
                case ConsoleKey.NumPad0:
                    Console.WriteLine("\nShutting down...");
                    _currentState = ExperimentState.Exit;
                    break;
                case ConsoleKey.D1:
                case ConsoleKey.NumPad1:
                    Console.WriteLine("\nRouting to Alignment Mode...");
                    _currentState = ExperimentState.Alignment;
                    break;
                case ConsoleKey.D2:
                case ConsoleKey.NumPad2:
                    Console.WriteLine("\nRouting to Parameter Input...");
                    _expParams.StartPos = _device.Position;
                    _currentState = ExperimentState.ParameterInput;
                    break;
                case ConsoleKey.D3:
                case ConsoleKey.NumPad3:
                    _currentState = ExperimentState.ParameterInput;
                    break;
                case ConsoleKey.D4:
                case ConsoleKey.NumPad4:
                    _currentState = ExperimentState.Running;
                    break;
                default:
                    Console.WriteLine("\n[Error] Invalid selection. Choose a valid menu option.");
                    Thread.Sleep(1500);
                    break;                    
            }
        }

        #endregion
        #region Soft Stop
        static void RunSoftStopMenu()
        {
            _softStop = false;
            Console.WriteLine("\n==============================");
            Console.WriteLine("        INTERRUPT MENU        ");
            Console.WriteLine("==============================");

            if (_currentState == ExperimentState.ParameterInput)
            {
                Console.WriteLine("[0] Resume inputs");
                Console.WriteLine("[1] Abandon inputs & return to Alignment Mode (No Re-Homing)");
                Console.WriteLine("[2] Clear inputs & restart Configuration Mode");
            }
            else if (_currentState == ExperimentState.Running)
            {
                Console.WriteLine("[0] Resume experiment");
                Console.WriteLine("[1] Terminate run immediately & save collected data");
                Console.WriteLine("[2] Abort run & re-enter Alignment Mode at CURRENT position");
                Console.WriteLine("[3] Abort run, return to START position, & re-enter Alignment Mode at START position");
            }
            Console.WriteLine("\nSelect an option: ");
            ConsoleKey chosenKey = ConsoleKey.NoName;

            while (_keepRunning) //Allow for emergency stop while waiting
            {
                if (Console.KeyAvailable)
                {
                    var keyInfo = Console.ReadKey(true);
                    chosenKey = keyInfo.Key;
                    break;
                }
                Thread.Sleep(20);
            }

            if (!_keepRunning){ _currentState = ExperimentState.Exit; return; }

            if (_currentState == ExperimentState.ParameterInput)
            {
                switch (chosenKey)
                {
                    case ConsoleKey.D0:
                    case ConsoleKey.NumPad0:
                        Console.WriteLine("\nResuming configuration...");
                        break;
                    case ConsoleKey.D1:
                    case ConsoleKey.NumPad1:
                        _currentState = ExperimentState.Alignment;
                        break;
                    case ConsoleKey.D2:
                    case ConsoleKey.NumPad2:
                        _currentState = ExperimentState.ParameterInput;
                        break;
                    default:
                        Console.WriteLine("\nInvalid selection. Resuming configuration...");
                        break;
                }
            }
            else if (_currentState == ExperimentState.Running)
            {
                switch (chosenKey)
                {
                    case ConsoleKey.D0:
                    case ConsoleKey.NumPad0:
                        Console.WriteLine("\nResuming experiment...");
                        break;
                    case ConsoleKey.D1:
                    case ConsoleKey.NumPad1:
                        _currentState = ExperimentState.PostExperiment;
                        break;
                    case ConsoleKey.D2:
                    case ConsoleKey.NumPad2:
                        _currentState = ExperimentState.Alignment;
                        break;
                    case ConsoleKey.D3:
                    case ConsoleKey.NumPad3:
                        Console.WriteLine("\nReturning motor to starting alignment position...");
                        MotorDirection dir = _expParams.StepSize >= 0 ? MotorDirection.Backward : MotorDirection.Forward;
                        decimal returnDist = Math.Abs(_device.Position - _expParams.StartPos);
                        PerformSafeMove(dir, returnDist);
                        _currentState = ExperimentState.Alignment;
                        break;
                    default:
                        Console.WriteLine("\nInvalid selection. Resuming experiment...");
                        break;
                }
            }
        }
        #endregion
    }
    #region Logger
    public static class SessionLogger
    {
        private static string _filePath;
        public static int _slicesLogged = 0;
        
        public static void Initialize(decimal step, int slices, int interval)
        {
            //create a logs directory folder
            string logFolder = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Logs");
            if (!Directory.Exists(logFolder)) { Directory.CreateDirectory(logFolder); }
            //create file name
            string timestamp = DateTime.Now.ToString("yyy-MM-dd_HH-mm");
            _filePath = Path.Combine(logFolder, $"Log_{timestamp}.csv");
            //top section: experiment summary
            StringBuilder sb = new StringBuilder();
            sb.AppendLine("[Experiment Summary]");
            sb.AppendLine($"Date,{DateTime.Now:yyy-MM-dd_HH:mm}");
            sb.AppendLine($"Step Size (mm),{step}");
            sb.AppendLine($"Total Slices,{slices}");
            sb.AppendLine($"Interval (ms),{interval}");
            sb.AppendLine("---");
            //headers for LogSlice()
            sb.AppendLine("Slice,Start_Time (HH:mm:ss.ff),End_Time (HH:mm:ss.ff),Pos (mm)");

            File.WriteAllText(_filePath, sb.ToString());
        }

        public static void LogSlice(string slice, DateTime startTime, DateTime endTime, decimal Pos)
        {
            string startTimeStr = startTime.ToString("HH:mm:ss.ff");
            string endTimeStr = endTime.ToString("HH:mm:ss.ff");
            string line = $"{slice},{startTimeStr},{endTimeStr},{Pos:F4}\n";
            File.AppendAllText(_filePath, line);
            _slicesLogged++;
        }

        public static void FinalizeLogFile()
        {
            if (string.IsNullOrEmpty(_filePath) || !File.Exists(_filePath))
            {
                Console.WriteLine("[Log] No active log file found to finalize.");
                return;
            }

            try
            {
                StringBuilder footer = new StringBuilder();
                footer.AppendLine("-------------------------------");
                footer.AppendLine($"Experiment Ended : {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
                footer.AppendLine($"Total Slices : {_slicesLogged}");
                footer.AppendLine("-------------------------------");
                File.AppendAllText(_filePath, footer.ToString());

                Console.WriteLine($"[Log] Log file finalized at: {_filePath}");
            }
            catch (Exception ex)
            {
                Console.WriteLine($"\n[LOG ERROR] Could not finalize file: {ex.Message}");
            }
            finally
            {
                _filePath = null; //clear file path if user runs another exp
                _slicesLogged = 0;
            }
        }
    }
    #endregion
    public class StepwiseParameters
    {
        public decimal StartPos { get; set; }
        public decimal StepSize { get; set; }
        public decimal FinishPos { get; set; }
        public int TotalSlices { get; set; }
        public int IntervalMs { get; set; }
    }
}
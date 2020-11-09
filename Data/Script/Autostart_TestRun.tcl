namespace eval RL_Driver_Helper {

proc Check_loaded_TestRun {{FName Racetrack_1}} {
	global TestRun
	
	if { $TestRun(FName) eq $FName  } {
		Log "Testrun already open: $FName"
		return 1
	} \
	\
	elseif { $TestRun(FName) ne $FName } {
		LoadTestRun $FName 1
	}
}

# Load TestRun IFile
Check_loaded_TestRun

while 0 {
	# Start Sim first
	StartSim


	# Wait until stopped
	set establ [WaitForStatus idle]
	if { $establ == -1 } { Log "TestRun was longer than 1000 seconds." } \
	else { Log "Sim idle" }
}


}; # Namespace end

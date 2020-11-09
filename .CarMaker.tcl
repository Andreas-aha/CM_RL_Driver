set Pgm(TcpCmdPort) 14100


proc Load_Autostart_TestRun {} {
	after idle {source Data/Script/Autostart_TestRun.tcl}
}

Load_Autostart_TestRun

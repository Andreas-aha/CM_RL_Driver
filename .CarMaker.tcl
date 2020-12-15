#set Pgm(TcpCmdPort) 14100


proc Load_Autostart_TestRun {} {
	after idle {source Data/Script/Autostart_TestRun.tcl}
}

proc Load_Reset_RTFac {} {
	after idle {source Data/Script/Reset_RTFac.tcl}
}

Load_Autostart_TestRun
Load_Reset_RTFac


set Ovrwrite_RTFac 0

proc MySetSimTimeAcc RTFac {

    variable Ovrwrite_RTFac

    if { !$Ovrwrite_RTFac } {
        SetSimTimeAcc $RTFac
    } else {
        SetSimTimeAcc $Ovrwrite_RTFac
    }

}
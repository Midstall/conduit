DefinitionBlock ("", "DSDT", 2, "MID", "CONDUIT", 0x00000001)
{
    Scope (\_SB)
    {
        Device (COM0)
        {
            Name (_HID, EisaId ("PNP0501"))
            Name (_CRS, ResourceTemplate ()
            {
                Memory32Fixed (ReadWrite, 0x10000000, 0x00001000)
                Interrupt (ResourceConsumer, Level, ActiveHigh, Exclusive) { 10 }
            })
        }
    }
}

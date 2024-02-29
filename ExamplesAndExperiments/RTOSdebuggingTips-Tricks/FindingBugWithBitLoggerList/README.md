Use the void SetBitToLoggerList(uint32_t inputBITx, BitLoggerList_t* BitLogger) for searching
"UnHappyBugFlags" or code sections where while(...) -like ops being used which causes the task stucking.

/*This func. implemented for debugging Multi Thread systems where sometimes we're stuck
on while(someUnhappyFlag){;;;} or on do{;;;}while(someUnhappyFlag) -like operations and
for us it seemed that our task where while(...) ops located is not allocated even or it
has been deleted or corrupted or stack/memory overflow allocated. This func. helps to not confuse
and first of all where while(...) -like ops located put this to there and use BitLoggerList(...)
on Timer Interrupt Handler section to know why some tasks are stucked and where. I hope you'll enjoy.
Look the example at main.c as a reference to guidance. I admit that its not deeply and confidently
tested. But it is for while I hope!*/
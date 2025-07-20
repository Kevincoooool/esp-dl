@for /f "delims=" %%i in ('dir build /ad /s /b') do @rd /s /q %%i
@for /f "delims=" %%i in ('dir managed_components /ad /s /b') do @rd /s /q %%i
exit

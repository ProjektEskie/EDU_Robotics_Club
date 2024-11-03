SET var=%cd%
cd var
rmdir /s /q .\build
rmdir /s /q .\dist
nicegui-pack --onefile --windowed --name "RCC" main.py
del ../RCC.exe
move ./dist/RCC.exe ../
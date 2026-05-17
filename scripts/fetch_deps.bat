@echo off
setlocal EnableDelayedExpansion

:: Fetch all external dependencies for kigstudio
:: Run this from the project root directory

cd /d "%~dp0\.."
set "ROOT=%CD%"

echo ==========================================
echo Fetching dependencies for kigstudio
echo Root: %ROOT%
echo ==========================================

:: 1. Update Git submodules
echo.
echo [1/4] Updating Git submodules...
git submodule update --init --recursive
if errorlevel 1 (
    echo ERROR: Failed to update submodules.
    exit /b 1
)

:: 2. Download Eigen3 (header-only)
echo.
echo [2/4] Checking Eigen3...
if exist "%ROOT%\dep\eigen3\Eigen\Core" (
    echo Eigen3 already exists, skipping.
) else (
    echo Downloading Eigen3 3.4.0...
    if exist "%ROOT%\dep\eigen3" rmdir /s /q "%ROOT%\dep\eigen3"
    curl -L -o "%ROOT%\dep\eigen-3.4.0.tar.gz" "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz"
    if errorlevel 1 (
        echo ERROR: Failed to download Eigen3.
        exit /b 1
    )
    tar -xzf "%ROOT%\dep\eigen-3.4.0.tar.gz" -C "%ROOT%\dep"
    move "%ROOT%\dep\eigen-3.4.0" "%ROOT%\dep\eigen3"
    del "%ROOT%\dep\eigen-3.4.0.tar.gz"
    echo Eigen3 ready.
)

:: 3. Download Boost and generate headers
echo.
echo [3/4] Checking Boost...
if exist "%ROOT%\dep\boost\boost\version.hpp" (
    echo Boost already exists, skipping.
) else (
    echo Downloading Boost 1.82.0...
    if exist "%ROOT%\dep\boost" rmdir /s /q "%ROOT%\dep\boost"
    curl -L -o "%ROOT%\dep\boost_1_82_0.tar.gz" "https://github.com/boostorg/boost/releases/download/boost-1.82.0/boost-1.82.0.tar.gz"
    if errorlevel 1 (
        echo ERROR: Failed to download Boost.
        exit /b 1
    )
    tar -xzf "%ROOT%\dep\boost_1_82_0.tar.gz" -C "%ROOT%\dep"
    move "%ROOT%\dep\boost-1.82.0" "%ROOT%\dep\boost"
    del "%ROOT%\dep\boost_1_82_0.tar.gz"

    echo Running Boost bootstrap...
    cd /d "%ROOT%\dep\boost"
    call bootstrap.bat
    if errorlevel 1 (
        echo ERROR: Boost bootstrap failed.
        exit /b 1
    )

    echo Generating Boost headers...
    b2 headers
    if errorlevel 1 (
        echo ERROR: Boost header generation failed.
        exit /b 1
    )
    cd /d "%ROOT%"
    echo Boost ready.
)

:: 4. Verify CGAL (should already be present as a submodule)
echo.
echo [4/4] Checking CGAL...
if exist "%ROOT%\dep\cgal\Installation\CGALConfig.cmake" (
    echo CGAL found.
) else (
    echo WARNING: CGAL not found. Make sure the submodule is initialized.
    echo   git submodule update --init --recursive dep/cgal
)

echo.
echo ==========================================
echo All dependencies fetched successfully!
echo ==========================================
endlocal

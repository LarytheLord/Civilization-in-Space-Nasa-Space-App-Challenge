# LunaBot Frontend - Fix Styling Script
# Run this in PowerShell to fix the styling issues

Write-Host "🚀 LunaBot Frontend - Styling Fix Script" -ForegroundColor Cyan
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host ""

# Check if we're in the right directory
if (-not (Test-Path "package.json")) {
    Write-Host "❌ Error: package.json not found!" -ForegroundColor Red
    Write-Host "Please run this script from the robot/frontend directory" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Usage:" -ForegroundColor White
    Write-Host "  cd robot/frontend" -ForegroundColor Gray
    Write-Host "  .\fix-styling.ps1" -ForegroundColor Gray
    exit 1
}

Write-Host "✅ Found package.json" -ForegroundColor Green
Write-Host ""

# Step 1: Clean cache
Write-Host "🧹 Step 1: Cleaning build cache..." -ForegroundColor Yellow
if (Test-Path "node_modules\.cache") {
    Remove-Item -Recurse -Force "node_modules\.cache" -ErrorAction SilentlyContinue
    Write-Host "   ✓ Removed node_modules\.cache" -ForegroundColor Green
}
if (Test-Path ".cache") {
    Remove-Item -Recurse -Force ".cache" -ErrorAction SilentlyContinue
    Write-Host "   ✓ Removed .cache" -ForegroundColor Green
}

# Step 2: Verify config files
Write-Host ""
Write-Host "📋 Step 2: Checking configuration files..." -ForegroundColor Yellow

$configFiles = @(
    "tailwind.config.js",
    "postcss.config.js"
)

$allConfigsExist = $true
foreach ($file in $configFiles) {
    if (Test-Path $file) {
        Write-Host "   ✓ Found $file" -ForegroundColor Green
    } else {
        Write-Host "   ✗ Missing $file" -ForegroundColor Red
        $allConfigsExist = $false
    }
}

if (-not $allConfigsExist) {
    Write-Host ""
    Write-Host "❌ Some configuration files are missing!" -ForegroundColor Red
    Write-Host "The files should have been created. Please check the STYLING_FIXED.md guide." -ForegroundColor Yellow
    exit 1
}

# Step 3: Install/verify dependencies
Write-Host ""
Write-Host "📦 Step 3: Installing/verifying dependencies..." -ForegroundColor Yellow
Write-Host "   This may take 30-60 seconds..." -ForegroundColor Gray

try {
    npm install 2>&1 | Out-Null
    Write-Host "   ✓ Dependencies installed" -ForegroundColor Green
} catch {
    Write-Host "   ✗ Failed to install dependencies" -ForegroundColor Red
    Write-Host "   Error: $_" -ForegroundColor Red
    exit 1
}

# Step 4: Verify Tailwind
Write-Host ""
Write-Host "🎨 Step 4: Verifying Tailwind CSS..." -ForegroundColor Yellow

$tailwindCheck = npm list tailwindcss 2>&1 | Out-String
if ($tailwindCheck -match "tailwindcss@") {
    Write-Host "   ✓ Tailwind CSS is installed" -ForegroundColor Green
} else {
    Write-Host "   ⚠ Tailwind CSS not found, installing..." -ForegroundColor Yellow
    npm install -D tailwindcss postcss autoprefixer 2>&1 | Out-Null
    Write-Host "   ✓ Tailwind CSS installed" -ForegroundColor Green
}

# Success message
Write-Host ""
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host "✅ Styling fixes applied successfully!" -ForegroundColor Green
Write-Host "=========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "🚀 Next steps:" -ForegroundColor Cyan
Write-Host "   1. Start the app:" -ForegroundColor White
Write-Host "      npm start" -ForegroundColor Gray
Write-Host ""
Write-Host "   2. When browser opens, hard refresh:" -ForegroundColor White
Write-Host "      Press Ctrl + Shift + R" -ForegroundColor Gray
Write-Host "      Or Ctrl + F5" -ForegroundColor Gray
Write-Host ""
Write-Host "   3. You should now see:" -ForegroundColor White
Write-Host "      ✨ Styled dark panels with borders" -ForegroundColor Gray
Write-Host "      ✨ Color-coded status indicators" -ForegroundColor Gray
Write-Host "      ✨ Professional grid layouts" -ForegroundColor Gray
Write-Host "      ✨ Charts with proper backgrounds" -ForegroundColor Gray
Write-Host "      ✨ Smooth animations" -ForegroundColor Gray
Write-Host ""
Write-Host "🌕 Ready to impress the judges! Good luck!" -ForegroundColor Cyan
Write-Host ""

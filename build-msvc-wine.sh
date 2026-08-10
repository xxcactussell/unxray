#!/usr/bin/env bash
set -euo pipefail

# ─── Настройки ───────────────────────────────────────────────────────────────
MSVC_DIR="${HOME}/.msvc/bin/x64"
SOLUTION="src/engine.sln"
CONFIGURATION="${1:-Release}"   # Debug | Mixed | Release | ReleaseMasterGold
PLATFORM="${2:-x64}"            # x64 | Win32
TOOLSET="v145"                  # соответствует MSVC 14.5x в msvc-wine
# NOTE: Wine Mono не поддерживает out-of-proc MSBuild узлы (/m > 1 вызывает
# TypeLoadException в дочерних процессах). Принудительно используем 1 поток.
JOBS=1
# ─────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# Проверяем msvc-wine
if [[ ! -x "${MSVC_DIR}/msbuild" ]]; then
    echo "[ERROR] msvc-wine не найден: ${MSVC_DIR}/msbuild"
    echo "        Убедитесь, что msvc-wine установлен в ~/.msvc"
    exit 1
fi

# Проверяем wine
if ! command -v wine &>/dev/null; then
    echo "[ERROR] wine не найден. Установите: sudo apt install wine"
    exit 1
fi

# wine64 нужен скрипту wine-msvc.sh — создаём симлинк если его нет
if ! command -v wine64 &>/dev/null; then
    mkdir -p "${HOME}/.local/bin"
    ln -sf "$(command -v wine)" "${HOME}/.local/bin/wine64"
    export PATH="${HOME}/.local/bin:${PATH}"
fi

# Проверяем winbind (нужен для mspdbsrv.exe под Wine)
if ! dpkg -l winbind 2>/dev/null | grep -q "^ii"; then
    echo "[WARN] winbind не установлен — возможны ошибки C1902 при параллельной компиляции."
    echo "       Установите: sudo apt install winbind"
    echo ""
fi

export PATH="${MSVC_DIR}:${HOME}/.local/bin:${PATH}"
export WINEDEBUG="-all"

echo "╔══════════════════════════════════════════════════════╗"
echo "║          msvc-wine / MSBuild — сборка проекта        ║"
echo "╠══════════════════════════════════════════════════════╣"
echo "║  Solution     : ${SOLUTION}"
echo "║  Configuration: ${CONFIGURATION}"
echo "║  Platform     : ${PLATFORM}"
echo "║  Toolset      : ${TOOLSET}"
echo "║  Jobs         : ${JOBS}"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# Убиваем старый wineserver (очищаем состояние mspdbsrv.exe)
wineserver -k 2>/dev/null || true
sleep 0.5

# ── NuGet restore ────────────────────────────────────────────────────────────
NUGET_EXE="${HOME}/nuget.exe"
if [[ ! -f "${NUGET_EXE}" ]]; then
    echo "[INFO] Скачиваем nuget.exe..."
    wget -q -O "${NUGET_EXE}" "https://dist.nuget.org/win-x86-commandline/latest/nuget.exe"
fi
echo "[INFO] Восстанавливаем NuGet пакеты..."
WINEDEBUG="-all" wine "${NUGET_EXE}" restore "${SOLUTION}" \
    -PackagesDirectory src/packages 2>&1 | grep -E "^(Restoring|Installed|Added|ERROR|Error)" || true
# ─────────────────────────────────────────────────────────────────────────────

START_TIME=$(date +%s)

msbuild \
    /p:Configuration="${CONFIGURATION}" \
    /p:Platform="${PLATFORM}" \
    /p:PlatformToolset="${TOOLSET}" \
    /m:"${JOBS}" \
    /nodeReuse:false \
    /v:minimal \
    /nologo \
    "${SOLUTION}"

EXIT_CODE=$?
END_TIME=$(date +%s)
ELAPSED=$(( END_TIME - START_TIME ))

echo ""
if [[ ${EXIT_CODE} -eq 0 ]]; then
    echo "✅ Сборка завершена успешно за ${ELAPSED}с"
    
    TARGET_DIR="build/${CONFIGURATION}/${PLATFORM}"
    echo "[INFO] Формируем релизную папку: ${TARGET_DIR}"
    
    mkdir -p "${TARGET_DIR}/bin"
    
    echo "  - Копируем исполняемые файлы..."
    rsync -a --exclude="*.pdb" "bin/${PLATFORM}/${CONFIGURATION}/" "${TARGET_DIR}/bin/"
    
    echo "  - Подменяем старый OpenAL32.dll на OpenAL Soft (soft_oal.dll)..."
    if [[ -f "${TARGET_DIR}/bin/soft_oal.dll" ]]; then
        mv -f "${TARGET_DIR}/bin/soft_oal.dll" "${TARGET_DIR}/bin/OpenAL32.dll"
    fi
    
    echo "  - Копируем внешние зависимости (SDL3)..."
    cp "Externals/SDL3/lib/${PLATFORM}/SDL3.dll" "${TARGET_DIR}/bin/" 2>/dev/null || true
    
    if [[ -d "res/gamedata" ]]; then
        echo "  - Копируем gamedata..."
        rsync -a "res/gamedata" "${TARGET_DIR}/"
    fi
    
    if [[ -f "res/fsgame.ltx" ]]; then
        echo "  - Копируем fsgame.ltx..."
        cp "res/fsgame.ltx" "${TARGET_DIR}/"
    fi
    
    echo "   Артефакты готовы в: ${TARGET_DIR}/"
else
    echo "❌ Сборка завершилась с ошибкой (код ${EXIT_CODE})"
fi

exit ${EXIT_CODE}

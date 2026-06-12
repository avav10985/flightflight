# push_docs.ps1 — 一鍵推送 docs 變更(報告網站)
# 用法:.\push_docs.ps1            (自動帶時間戳訊息)
#       .\push_docs.ps1 "訊息"    (自訂 commit 訊息)
param([string]$Msg = "")

Push-Location $PSScriptRoot
try {
    $changes = git status --porcelain docs
    if ([string]::IsNullOrWhiteSpace($changes)) {
        Write-Output "docs/ 沒有變更,不用推。"
        exit 0
    }
    Write-Output "偵測到變更:`n$changes"
    if ([string]::IsNullOrWhiteSpace($Msg)) {
        $Msg = "docs: 更新報告內容 $(Get-Date -Format 'yyyy-MM-dd HH:mm')"
    }
    git add docs
    git commit -m $Msg
    if ($LASTEXITCODE -ne 0) { Write-Output "Commit 失敗。"; exit 1 }
    git push origin main
    if ($LASTEXITCODE -eq 0) { Write-Output "推送成功。" } else { Write-Output "推送失敗(先 git pull?)。"; exit 1 }
} finally {
    Pop-Location
}

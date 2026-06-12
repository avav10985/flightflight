# Push docs changes: add, commit if changes, push to origin/main
Push-Location $PSScriptRoot
try {
    $changes = git status --porcelain
    if ([string]::IsNullOrWhiteSpace($changes)) {
        Write-Output "No changes to commit in the repository."
        exit 0
    }
    # Show what will be committed
    Write-Output "Changes detected:\n$changes"
    git add docs
    git commit -m "docs: make QR code persistent in top-right on all pages"
    if ($LASTEXITCODE -ne 0) {
        Write-Output "Commit failed or nothing to commit. Aborting."
        exit 1
    }
    git push origin main
    if ($LASTEXITCODE -eq 0) { Write-Output "Push succeeded." } else { Write-Output "Push failed."; exit 1 }
} finally {
    Pop-Location
}

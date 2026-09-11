$ErrorActionPreference = 'Stop'
$templateRoot = Split-Path -Parent $PSScriptRoot
$templatePath = [System.IO.Path]::GetFullPath((Join-Path $templateRoot '国赛2026全文论文模板.docx'))
$previewPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot 'qa/template-preview.pdf'))
$wordInstance = $null
$openedDocument = $null
try {
    $wordInstance = New-Object -ComObject Word.Application
    $wordInstance.Visible = $false
    $wordInstance.DisplayAlerts = 0
    $openedDocument = $wordInstance.Documents.Open($templatePath, $false, $true)
    $openedDocument.Repaginate()
    $openedDocument.ExportAsFixedFormat($previewPath, 17)
    Write-Output ('Pages: ' + $openedDocument.ComputeStatistics(2))
    Write-Output $previewPath
} finally {
    if ($null -ne $openedDocument) {
        $openedDocument.Close(0)
        [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($openedDocument)
    }
    if ($null -ne $wordInstance) {
        $wordInstance.Quit()
        [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($wordInstance)
    }
}

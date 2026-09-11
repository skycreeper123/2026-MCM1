$ErrorActionPreference = 'Stop'
$q1DocPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../Q1正文.docx'))
$q1PdfPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot 'Q1-preview.pdf'))
$q1Word = $null
$q1Opened = $null
try {
    $q1Word = New-Object -ComObject Word.Application
    $q1Word.Visible = $false
    $q1Word.DisplayAlerts = 0
    $q1Opened = $q1Word.Documents.Open($q1DocPath, $false, $true)
    $q1Opened.Repaginate()
    $q1Opened.ExportAsFixedFormat($q1PdfPath, 17)
    Write-Output ('Pages: ' + $q1Opened.ComputeStatistics(2))
} finally {
    if ($null -ne $q1Opened) { $q1Opened.Close(0); [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($q1Opened) }
    if ($null -ne $q1Word) { $q1Word.Quit(); [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($q1Word) }
}

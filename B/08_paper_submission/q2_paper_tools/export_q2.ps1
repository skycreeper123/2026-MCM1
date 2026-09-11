$ErrorActionPreference = 'Stop'
$q2DocPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../Q2正文.docx'))
$q2PdfPath = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot 'Q2-preview.pdf'))
$q2Word = $null
$q2Opened = $null
try {
    $q2Word = New-Object -ComObject Word.Application
    $q2Word.Visible = $false
    $q2Word.DisplayAlerts = 0
    $q2Opened = $q2Word.Documents.Open($q2DocPath, $false, $true)
    $q2Opened.Repaginate()
    $q2Opened.ExportAsFixedFormat($q2PdfPath, 17)
    Write-Output ('Pages: ' + $q2Opened.ComputeStatistics(2))
} finally {
    if ($null -ne $q2Opened) { $q2Opened.Close(0); [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($q2Opened) }
    if ($null -ne $q2Word) { $q2Word.Quit(); [void][System.Runtime.InteropServices.Marshal]::FinalReleaseComObject($q2Word) }
}

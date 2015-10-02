REM This script will oversample a mesh, saving only the vertices to a new file.

REM Guidance from the following website:
REM http://www.andrewhazelden.com/blog/2012/04/automate-your-meshlab-workflow-with-mlx-filter-scripts/

set input_mesh=%USERPROFILE%\Documents\input_mesh.ply
set oversampled_cloud=%USERPROFILE%\Documents\oversampled_cloud.ply

cd "C:\Program Files\VCG\MeshLab\"

meshlabserver.exe -i "%input_mesh%" -o "%oversampled_cloud%" -s %USERPROFILE%\Documents\GitHub\apc\library_generation\oversample.mlx -om vc vn

REM pause
cmd /k

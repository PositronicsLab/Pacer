function h = heightmap
close all;
NUM_HILLS           = 1000;
SIZE                = 3;
RES                 = 0.02; 
HEIGHT              = 0.02;
HILL_WIDTH          = 0.01; 
WIDTH_VARIABILITY   = 0.5; % 0 =< x < 1
HEIGHT_VARIABILITY  = 0;

[x,y] = meshgrid(0:RES:SIZE,0:RES:SIZE);

h = zeros(size(x));
for i = 1:NUM_HILLS
    % MEAN
    mu = rand(1,2)*SIZE;

    % COV
    sigma = diag([HILL_WIDTH + rand*HILL_WIDTH*WIDTH_VARIABILITY,HILL_WIDTH + rand*HILL_WIDTH*WIDTH_VARIABILITY]- 0.5*HILL_WIDTH*WIDTH_VARIABILITY);
    hill = gauss2d(x,y, sigma, mu);
%     hill = hill .* (HEIGHT+randn*HEIGHT*HEIGHT_VARIABILITY)/max(max(hill));
    hill = hill .* (1 + randn*HEIGHT_VARIABILITY/max(max(hill)));
    h = h + hill;

end
h = h - min(min(h));
% Normalize height to parameter
h = h .* (HEIGHT/max(max(h)));
surf(x,y,h);

 axis([0 SIZE 0 SIZE 0 5*max(max(h))]);
 heightmap_size = size(h);
 save('heightmap.mat','h','-ascii')
 prepend2file([num2str(heightmap_size(1)),' ',num2str(heightmap_size(2))],'heightmap.mat',1);
 
%  h2 = fliplr(h);
%  h2 = -h2;
%   save('heightmap-viz.mat','h2','-ascii')
%  prepend2file([num2str(heightmap_size(1)),' ',num2str(heightmap_size(2))],'heightmap-viz.mat',1);

end

function mat = gauss2d(R,C, SIGMA, MU)
    gsize = size(R);
    mat = mvnpdf([R(:),C(:)],MU,SIGMA);
    mat = reshape(mat,gsize(1),gsize(2));
end

function prepend2file( string, filename, newline )
% newline:  is an optional boolean, that if true will append a \n to the end 
% of the string that is sent in such that the original text starts on the 
% next line rather than the end of the line that the string is on 
% string:  a single line string 
% filename:  the file you want to prepend to 
      tempFile = tempname
      fw = fopen( tempFile, 'wt' );
      if nargin < 3
          newline = false;
      end
      if newline
          fwrite( fw, sprintf('%s\n', string ) );
      else
          fwrite( fw, string );
      end
      fclose( fw );
      appendFiles( filename, tempFile );
      copyfile( tempFile, filename );
      delete(tempFile);
end

% append readFile to writtenFile
function status = appendFiles( readFile, writtenFile )
      fr = fopen( readFile, 'rt' );
      fw = fopen( writtenFile, 'at' );
      while feof( fr ) == 0
          tline = fgetl( fr );
          fwrite( fw, sprintf('%s\n',tline ) );
      end
      fclose(fr);
      fclose(fw);
end
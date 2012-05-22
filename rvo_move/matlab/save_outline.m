close all;
clear all;
addpath('../maps');
map = Map(imread('levine_towne_nav.pgm'), 0.05, [-12.55, -17.63], [0 0]);

%%
inflated_map = map.InflateObstacles(1.0);

polygons = fit_polygons(inflated_map, 120, 600);
% Change coordinates of outside boundary to be counterclockwise
polygons{1} = flipud(polygons{1});

%%
min_sep = 0.8;
max_sep = 1.5;
smoothed = cellfun(@(x) poly_smooth(x, min_sep, max_sep), polygons, 'UniformOutput', false);
clf(gcf);
figure(gcf);
plot_poly(smoothed)
%%
dt = delauncy_decomp(smoothed);
[xy, neighbors] = make_graph(map, dt);
assert(~any(map.collide(xy)));
%% Visualize results
map2 = Map(imread('levine_towne_nav.pgm'), 0.05, [-12.55, -17.63], [0 0]);
plot_delaunay(map2, dt, xy, neighbors)

%% Save the neighbors to disk
fid = fopen('neighbors.txt', 'w');
strs = cellfun(@(a) sprintf(' %i', a), neighbors, 'UniformOutput', false);
for k = 1:size(xy, 1)
    fprintf(fid, '%0.5f %0.5f%s\n', xy(k, :), strs{k});
end
fclose(fid);

%% Save the polygons to disk
fid = fopen('polygons.txt', 'w');
for k = 1:length(polygons)
    fprintf(fid, '===\n');
    fprintf(fid, '% 7.3f % 7.3f\n', polygons{k}.');
end
fprintf(fid, '===');
fclose(fid);

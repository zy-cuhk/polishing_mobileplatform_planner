clc,clear all,close all;
tic;

%% obtaining the input
data=load('second_scan_data/second_scan_data1.mat','room_vertices','room_plane_norm_vector','room_plane_edge_cell','room_plane_edge_centroid','room_plane_triangle_edge_cell','room_plane_triangle_cell');
room_plane_norm_vector1=data.room_plane_norm_vector;
for i=1:1:size(room_plane_norm_vector1,2)
    n1=room_plane_norm_vector1{i}(1,1);
    n2=sign(room_plane_norm_vector1{i}(1,2))*sqrt(1-n1^2);
    n3=0;
    room_plane_norm_vector{i}(1,1)=n1;
    room_plane_norm_vector{i}(1,2)=n2;
    room_plane_norm_vector{i}(1,3)=n3;
end
room_plane_edge_cell=data.room_plane_edge_cell;
room_plane_edge_centroid=data.room_plane_edge_centroid;
room_plane_triangle_edge_cell=data.room_plane_triangle_edge_cell;
room_plane_triangle_cell=data.room_plane_triangle_cell;
room_vertices=data.room_vertices;

%% painting process parameters are listed as follows:
%% the adjustable parameters can be: painting_gun_to_wall_distance and painting_ellipse_long_axis_length
painting_gun_to_wall_distance=0.31;
painting_ellipse_long_axis_length=0.40;
painting_ellipse_short_axis_length=0.10;
painting_path_interval=painting_ellipse_long_axis_length*2/3;
waypoints_interval=painting_ellipse_short_axis_length/2;

%% generating renovation and mobile base planes 
distance_from_painting_gun=0.20;
panning_distance1=painting_gun_to_wall_distance+distance_from_painting_gun;
[renovation_plane_edge_cell,renovation_plane_norm_vector,renovation_plane_triangle_edge_cell]=room_panning_planes_generation(room_plane_norm_vector,room_plane_edge_cell,room_plane_triangle_edge_cell,panning_distance1);
manipulatorendeffector2base_distance=0.60;
panning_distance2=painting_gun_to_wall_distance+distance_from_painting_gun+manipulatorendeffector2base_distance;
[manipulatorbase_plane_edge_cell,manipulatorbase_plane_norm_vector,manipulatorbase_plane_triangle_edge_cell]=room_panning_planes_generation(room_plane_norm_vector,room_plane_edge_cell,room_plane_triangle_edge_cell,panning_distance2);
renovation_planes_visualization(room_plane_edge_cell, renovation_plane_edge_cell,manipulatorbase_plane_edge_cell);

%% generating renovation waypoints and waypaths
[renovation_effective_waypoints,renovation_effective_waypaths,room_plane_boundary,distance_waypoints2wallboundary_direction1,distance_waypoints2wallboundary_direction2]=renovation_planes_waypoint_generation(room_plane_edge_cell,room_plane_norm_vector,room_vertices,room_plane_triangle_cell,waypoints_interval,painting_path_interval,panning_distance1);
distance_waypoints2wallboundary_direction1
distance_waypoints2wallboundary_direction2
renovation_planes_waypoint_visualization(renovation_effective_waypoints,room_plane_edge_cell,renovation_plane_edge_cell,renovation_effective_waypaths);

%% generating renovation cells containing waypaths 
cell_length=0.55;
cell_width=1.3;

% [renovation_cells_clustering_waypaths,renovation_cells_waypioints_onwaypath,renovation_cells_manipulatorbase_positions, manipulator_endeffector_positions_onpath]=renovation_cells_generation(room_plane_edge_cell,renovation_plane_edge_cell,manipulatorbase_plane_norm_vector,renovation_effective_waypaths,renovation_plane_norm_vector,cell_length,cell_width,painting_path_interval);
% renovation_cells_waypath_visualization(renovation_cells_waypioints_onwaypath,renovation_cells_manipulatorbase_positions,renovation_plane_edge_cell,room_plane_edge_cell);



toc;



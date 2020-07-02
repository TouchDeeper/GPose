# data

## model_data
save the offline generation model data

## scene_data
save the test scene data
- scene_data/[model_name]/*/[scene_name].txt: correspondence_view_index se3_vector.

The se3_vector is model_T_scene that we think is right result. For simulation, we get the model_T_scene from gazebo. For real scene, we set the model_T_scene from the pose estimation result that we think is ideal. 

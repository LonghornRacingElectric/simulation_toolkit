from vehicle_model.suspension_model.suspension_data import SuspensionData
from vehicle_model.suspension_model.suspension import Suspension

test_sus = SuspensionData(path="./1_model_inputs/Nightwatch.yml")


# # Front Left
# FL_lower_inboard_fore = Node(position=[0.087376, 0.215900, 0.090000])
# FL_lower_inboard_aft = Node(position=[-0.095250, 0.215900, 0.090000])
# FL_lower_outboard = Node(position=[0, 0.556499, 0.124998])

# FL_upper_inboard_fore = Node(position=[0.086868, 0.215900, 0.200000])
# FL_upper_inboard_aft = Node(position=[-0.095250, 0.215900, 0.200000])
# FL_upper_outboard = Node(position=[-0.006347, 0.523240, 0.287020])

# FL_tie_inboard = Node(position=[0.041128, 0.215900, 0.117856])
# FL_tie_outboard = Node(position=[0.056000, 0.532333, 0.164821])

# FL_lower_pushrod = Node(position=[-0.01368377, 0.49897726, 0.30564884])
# FL_upper_pushrod = Node(position=[-0.04415429, 0.39924560, 0.40746267])
# FL_inboard_shock = Node(position=[-0.09276858, 0.24012839, 0.56990187])

# FL_contact_patch = Node(position=[0, 0.609600, 0])

# FL_spring_free_length = 1
# FL_spring_rate = 1

# # Rear Left
# RL_lower_inboard_fore = Node(position=[-1.298905, 0.282999, 0.090000])
# RL_lower_inboard_aft = Node(position=[-1.490977, 0.282999, 0.090000])
# RL_lower_outboard = Node(position=[-1.554983, 0.579999, 0.113030])

# RL_upper_inboard_fore = Node(position=[-1.298905, 0.282999, 0.217500])
# RL_upper_inboard_aft = Node(position=[-1.490977, 0.282999, 0.217500])
# RL_upper_outboard = Node(position=[-1.574797, 0.554998, 0.289560])

# RL_tie_inboard = Node(position=[-1.428059, 0.282999, 0.177800])
# RL_tie_outboard = Node(position=[-1.462710, 0.587375, 0.240741])

# RL_lower_pushrod = Node(position=[-1.54445540, 0.55586601, 0.13907432])
# RL_upper_pushrod = Node(position=[-1.47864722, 0.40618605, 0.29100344])
# RL_inboard_shock = Node(position=[-1.42763653, 0.29016283, 0.40877008])

# RL_contact_patch = Node(position=[-1.549400, 0.609600, 0])

# RL_spring_free_length = 1
# RL_spring_rate = 1

# # Rear Stabar
# Rr_stabar_left_arm_end = Node(position=[-1.295910, 0.309264, 0.353681])
# Rr_stabar_right_arm_end = Node(position=[-1.295974, -0.309264, 0.353397])
# Rr_stabar_left_droplink_end = Node(position=[-1.325579, 0.309297, 0.224467])
# Rr_stabar_right_droplink_end = Node(position=[-1.325758, -0.309116, 0.224467])
# Rr_stabar_bar_left_end = Node(position=[-1.249618, 0.309264, 0.343043])
# Rr_stabar_bar_right_end = Node(position=[-1.249618, -0.309264, 0.343043])
# Rr_stabar_torsional_stiffness = 1

# suspension = Suspension(
#     FL_lower_inboard_fore,
#     FL_lower_inboard_aft,
#     FL_lower_outboard,
#     FL_upper_inboard_fore,
#     FL_upper_inboard_aft,
#     FL_upper_outboard,
#     FL_tie_inboard,
#     FL_tie_outboard,
#     FL_lower_pushrod,
#     FL_upper_pushrod,
#     FL_inboard_shock,
#     FL_contact_patch,
#     FL_spring_free_length,
#     FL_spring_rate,

#     RL_lower_inboard_fore,
#     RL_lower_inboard_aft,
#     RL_lower_outboard,
#     RL_upper_inboard_fore,
#     RL_upper_inboard_aft,
#     RL_upper_outboard,
#     RL_tie_inboard,
#     RL_tie_outboard,
#     RL_lower_pushrod,
#     RL_upper_pushrod,
#     RL_inboard_shock,
#     RL_contact_patch,
#     RL_spring_free_length,
#     RL_spring_rate,
    
#     Rr_stabar_left_arm_end,
#     Rr_stabar_right_arm_end,
#     Rr_stabar_left_droplink_end,
#     Rr_stabar_right_droplink_end,
#     Rr_stabar_bar_left_end,
#     Rr_stabar_bar_right_end,
#     Rr_stabar_torsional_stiffness,)

# suspension.generate_kin()
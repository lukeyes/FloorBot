import time


def calculate_cell_cost(history, current_time_ms, max_age_ms=60000):
    """
    Calculates the probabilistic cost of a cell based on history.

    Args:
        history: List of dicts [{'time': ms, 'occupied': True/False}]
        current_time_ms: The 'Now' timestamp
        max_age_ms: Time horizon (older than this = 0% confidence)

    Returns:
        cost: 0-254 (where 127 is Unknown/50%, 0 is Free, 254 is Lethal)
    """
    BASELINE_PROB = 0.5  # The "I don't know" state (50%)

    total_weighted_area = 0.0
    total_duration = 0.0

    # Analyze the timeline segment by segment
    for i in range(len(history)):
        obs = history[i]
        start_t = obs['time']

        # Determine end of this segment (either next observation or Now)
        if i + 1 < len(history):
            end_t = history[i + 1]['time']
        else:
            end_t = current_time_ms

        # Skip future/invalid segments
        if end_t > current_time_ms: end_t = current_time_ms
        if end_t <= start_t: continue

        duration = end_t - start_t

        # Calculate how old this segment is (using midpoint for accuracy)
        midpoint_t = start_t + (duration / 2)
        age = current_time_ms - midpoint_t

        # 1. Calculate Confidence (Linear Decay)
        # 1.0 = Brand new data, 0.0 = Ancient data
        confidence = 1.0 - (age / max_age_ms)
        if confidence < 0: confidence = 0.0

        # 2. Determine Raw State (1.0 = Occupied, 0.0 = Free)
        p_observed = 1.0 if obs['occupied'] else 0.0

        # 3. APPLY DRIFT FORMULA
        # Blend the Observation with the Baseline based on Confidence
        # If confidence is 0, we purely use Baseline (0.5)
        p_effective = (p_observed * confidence) + (BASELINE_PROB * (1.0 - confidence))

        # 4. Integrate
        total_weighted_area += (p_effective * duration)
        total_duration += duration

    # Handle empty history (pure unknown)
    if total_duration == 0:
        final_prob = BASELINE_PROB
    else:
        final_prob = total_weighted_area / total_duration

    # Map Probability (0.0 - 1.0) to Cost (0 - 254)
    # 0.0 -> 0 (Free)
    # 0.5 -> 127 (Unknown)
    # 1.0 -> 254 (Lethal)
    return int(final_prob * 254)


# ==========================================
# TEST SCENARIOS
# ==========================================
NOW = 100000  # Let's say system time is 100,000 ms
HORIZON = 10000  # 10 second memory for this test

print(f"{'SCENARIO':<30} | {'PROB':<6} | {'COST':<4} | {'MEANING'}")
print("-" * 65)

# Case 1: Freshly Blocked (Observed occupied 1ms ago)
# Should be near 100% (Cost 254)
hist_wall = [{'time': 99000, 'occupied': True}]
cost = calculate_cell_cost(hist_wall, NOW, HORIZON)
print(f"{'Fresh Wall (Recent)':<30} | {cost / 254:.2f}   | {cost:<4} | LEETHAL")

# Case 2: Freshly Free (Observed free 1ms ago)
# Should be near 0% (Cost 0)
hist_free = [{'time': 99000, 'occupied': False}]
cost = calculate_cell_cost(hist_free, NOW, HORIZON)
print(f"{'Fresh Space (Recent)':<30} | {cost / 254:.2f}   | {cost:<4} | FREE")

# Case 3: The "Grey Fog" (Observed Wall, but long ago)
# Data is from T=80,000 (20s ago). Horizon is 10s. Confidence is 0.
# Should drift back to 50% (Cost 127)
hist_old = [{'time': 80000, 'occupied': True}]
cost = calculate_cell_cost(hist_old, NOW, HORIZON)
print(f"{'Ancient Wall (Forgot)':<30} | {cost / 254:.2f}   | {cost:<4} | UNKNOWN")

# Case 4: Fading Memory (Observed Wall 5s ago, Horizon is 10s)
# Confidence is 0.5. Observed=1.0.
# Result should be (1.0 * 0.5) + (0.5 * 0.5) = 0.75
hist_fade = [{'time': 95000, 'occupied': True}]
cost = calculate_cell_cost(hist_fade, NOW, HORIZON)
print(f"{'Fading Wall (50% old)':<30} | {cost / 254:.2f}   | {cost:<4} | EXPENSIVE")
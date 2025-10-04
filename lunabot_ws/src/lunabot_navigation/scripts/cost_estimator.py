"""
Cost estimation for lunar habitat construction
"""

class CostEstimator:
    BASE_COST = 500_000_000  # $500M base cost
    
    @staticmethod
    def estimate(site_scores):
        """
        Calculate construction cost based on site conditions

        Args:
            site_scores: dict with keys 'safety', 'buildability', 'resources', 'expandability'

        Returns:
            dict with cost breakdown
        """
        # Terrain difficulty multiplier
        terrain_factor = 1 + (100 - site_scores['buildability']) / 100
        
        # Safety requirements multiplier
        safety_factor = 1 + (100 - site_scores['safety']) / 200
        
        # Additional costs
        foundation_cost = (100 - site_scores['buildability']) * 2_000_000
        radiation_shielding = (100 - site_scores['safety']) * 1_500_000
        
        initial_cost = (
            CostEstimator.BASE_COST * terrain_factor * safety_factor +
            foundation_cost + radiation_shielding
        )
        
        return {
            'initial': int(initial_cost),
            'expansion_5yr': int(initial_cost * 0.6),
            'annual_maintenance': int(initial_cost * 0.05),
            'breakdown': {
                'base': CostEstimator.BASE_COST,
                'terrain_adjustment': int(CostEstimator.BASE_COST * (terrain_factor - 1)),
                'safety_adjustment': int(CostEstimator.BASE_COST * (safety_factor - 1)),
                'foundation': int(foundation_cost),
                'shielding': int(radiation_shielding)
            }
        }
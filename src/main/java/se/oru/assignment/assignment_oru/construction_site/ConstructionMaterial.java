package se.oru.assignment.assignment_oru.construction_site;



import se.oru.assignment.assignment_oru.construction_site.util.constructionMaterialType.MATERIAL_TYPE;


public class ConstructionMaterial {
		
	
	protected MATERIAL_TYPE material_type;
	protected double fill_factor;
	
	public ConstructionMaterial(MATERIAL_TYPE material_type) {
		this.material_type = material_type;
	}
	
	public ConstructionMaterial(MATERIAL_TYPE material_type, double fill_factor) {
		this.material_type = material_type;
		this.fill_factor = fill_factor;
	}
	
	public void setFillFactor(double fill_factor) {
		this.fill_factor = fill_factor;
	}
	
	public double getFillFactor() {
		return this.fill_factor;
	}
}









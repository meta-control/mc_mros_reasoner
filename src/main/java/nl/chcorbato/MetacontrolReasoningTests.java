package nl.chcorbato;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.dlsyntax.renderer.DLSyntaxObjectRenderer;
import org.semanticweb.owlapi.formats.PrefixDocumentFormat;
import org.semanticweb.owlapi.io.OWLObjectRenderer;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyCreationException;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.SimpleConfiguration;

import com.clarkparsia.pellet.owlapiv3.PelletReasonerFactory;

/**
 * @author chcorbato
 *
 */
public class MetacontrolReasoningTests {
	private static final String ONTOLOGY_FILE = "file:/home/chcorbato/mros_ws/tomasys/metacontrol_unexmin.owl";
	private static final String ONTOLOGY_IRI = "http://www.semanticweb.org/chcorbato/ontologies/2018/metacontrol_unexmin";
	private static OWLObjectRenderer renderer = new DLSyntaxObjectRenderer();
	
	public static void main(String[] args) throws OWLOntologyCreationException {
		
		//prepare ontology and reasoner
        OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
        OWLOntology ontology = manager.loadOntologyFromOntologyDocument(IRI.create(ONTOLOGY_FILE));
        OWLReasonerFactory reasonerFactory = PelletReasonerFactory.getInstance();
        OWLReasoner reasoner = reasonerFactory.createReasoner(ontology, new SimpleConfiguration());
        OWLDataFactory factory = manager.getOWLDataFactory();
        PrefixDocumentFormat pm = manager.getOntologyFormat(ontology).asPrefixOWLOntologyFormat();
        pm.setDefaultPrefix(ONTOLOGY_IRI + "#");
		
        //get tomasys classes
        OWLClass componentState = factory.getOWLClass(":ComponentState", pm);
        OWLClass componentClass = factory.getOWLClass(":ComponentClass", pm);
        OWLClass function = factory.getOWLClass(":Function", pm);
        OWLClass objective = factory.getOWLClass(":Objective", pm);
        OWLClass functionDesign = factory.getOWLClass(":FunctionDesign", pm);
        OWLClass functionGrounding = factory.getOWLClass(":FunctionGrounding", pm);

        
        for (OWLNamedIndividual cs : reasoner.getInstances(componentState, false).getFlattened()) {
            System.out.println("person : " + renderer.render(cs));
        }
		
	}

}

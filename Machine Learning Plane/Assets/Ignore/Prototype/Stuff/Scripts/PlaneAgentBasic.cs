using System.Threading;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;


public class PlaneAgentBasic : Agent
{
    public float thrustPower = 50f;
    public float pitchSpeed = 10f;
    public float rollSpeed = 15f;
    public float yawSpeed = 5f; // Geschwindigkeit für Gieren
    float previousDistance = 9999;
    public float maxSpeed = 50;
    public float maxRotation = 50;
    private Rigidbody rb;

    public GameObject checkpoint; // Ziel-Checkpoint
    private int checkpointCounter = 0;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        // Setze das Flugzeug und die Umgebung zurück
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        transform.position = new Vector3(Random.Range(50,-50), 2.5f, -450); // Startposition
        transform.rotation = Quaternion.identity;
        checkpoint.transform.position = new Vector3(Random.Range(500,-500), Random.Range(50,250), Random.Range(500,-500));
        checkpoint.transform.localScale =  new Vector3(50, 50, 50);
        checkpointCounter = 0;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Beobachtungen sammeln: relative Position des Checkpoints
        sensor.AddObservation(transform.InverseTransformPoint(checkpoint.transform.position));
        sensor.AddObservation(Vector3.Distance(transform.position, checkpoint.transform.position));
        sensor.AddObservation(Vector3.Dot(transform.forward, (checkpoint.transform.position - transform.position).normalized));

        // Geschwindigkeit
        sensor.AddObservation(rb.linearVelocity);
        // Orientierung
        sensor.AddObservation(transform.rotation);
        sensor.AddObservation(transform.forward);
        sensor.AddObservation(transform.up); // Zusätzlich: Orientierung oben
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Aktionen umsetzen
        float pitch = actions.ContinuousActions[0]; // Nickbewegung
        float roll = actions.ContinuousActions[1];  // Rollen
        float yaw = actions.ContinuousActions[2];   // Gieren (Yaw)
        float thrust = actions.ContinuousActions[3]; // Schub
        if(rb.linearVelocity.magnitude > maxSpeed){
            rb.linearVelocity = rb.linearVelocity.normalized * maxSpeed;
        }
        if(rb.angularVelocity.magnitude > maxRotation){
            rb.angularVelocity = rb.angularVelocity.normalized * maxRotation;
        }
        
        rb.AddForce(Vector3.up * 250 * Time.deltaTime, ForceMode.Acceleration);
        
        rb.AddRelativeTorque(Vector3.right * pitch * pitchSpeed);    // Pitch
        rb.AddRelativeTorque(Vector3.back * roll * rollSpeed);       // Roll
        rb.AddRelativeTorque(Vector3.up * yaw * yawSpeed);           // Yaw
        rb.AddRelativeForce(Vector3.forward * thrust * thrustPower); // Schub

        float distanceToCheckpoint = Vector3.Distance(transform.position, checkpoint.transform.position);

        AddReward(100 / distanceToCheckpoint);

        if(distanceToCheckpoint > 2500){
            AddReward(-500);
            Debug.Log("Too far away");
            EndEpisode();
        }

        
        if (distanceToCheckpoint > previousDistance)
        {
            // Belohne den Agenten für Annäherung
            AddReward(-0.1f); // Kleinere negative Belohnung
        }
        previousDistance = distanceToCheckpoint;
        float rollAngle = Vector3.Angle(transform.up, Vector3.up); // Winkel zwischen oben und Welt oben
        if (rollAngle > 45) // Toleranz von 10 Grad
        {
            AddReward(-rollAngle * 0.001f); // Bestrafung proportional zur Abweichung
        }
        else
        {
            AddReward(0.001f); // Kleine Belohnung für aufrechte Haltung
        }

        Vector3 directionToCheckpoint = (checkpoint.transform.position - transform.position).normalized;

        float forwardAlignment = Vector3.Dot(transform.forward.normalized, directionToCheckpoint);

    // Belohne das Fliegen in Richtung des Checkpoints
        if (forwardAlignment > 0.75f) // Nahezu geradeaus (Wert nahe 1.0 bedeutet fast perfekt ausgerichtet)
        {
            AddReward(0.001f); // Kleine Belohnung für korrektes Fliegen
        }
        else
        {
            AddReward(-0.001f * (1.0f - forwardAlignment)); // Bestrafe, je weiter die Ausrichtung abweicht
        }
        
        if(transform.position.y > 500){
            AddReward(-100);
            Debug.Log("Too high up");
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manuelle Steuerung für Tests
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Vertical"); // Pitch mit W/S
        continuousActions[1] = Input.GetKey(KeyCode.Q) ? -1f : Input.GetKey(KeyCode.E) ? 1f : 0f; // Roll mit Q/E
        continuousActions[2] = Input.GetAxis("Horizontal"); // Yaw mit A/D
        continuousActions[3] = Input.GetKey(KeyCode.Space) ? 1f : 0f; // Schub mit Leertaste
    }

    void OnCollisionEnter(Collision collision)
    {
        if(collision.gameObject.CompareTag("Ground")){
            AddReward(-500.0f);
            EndEpisode();
            Debug.Log("Touched Ground");
        }
        else if(collision.gameObject.CompareTag("Plane")){
            AddReward(-500.0f);
            EndEpisode();
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if(other.gameObject.CompareTag("Checkpoint")){
            AddReward(1000.0f);
            checkpoint.transform.position = new Vector3(Random.Range(500,-500), Random.Range(50,250), Random.Range(500,-500));
            checkpoint.transform.localScale =  new Vector3(checkpoint.transform.localScale.x/2 + 1,checkpoint.transform.localScale.y/2 + 1,checkpoint.transform.localScale.z/2 + 1);
            checkpointCounter++;
            Debug.Log("Touched Checkpoint " + checkpointCounter);
        }
    }

    void OnCollisionStay(Collision other)
    {
        if(other.gameObject.CompareTag("Runway")){
            rb.AddForce(Vector3.up * 1500 * Time.deltaTime, ForceMode.Acceleration);
            rb.AddForce(Vector3.forward * 100 * Time.deltaTime, ForceMode.Acceleration);
            //AddReward(-0.000001f);

            Vector3 contactNormal = other.GetContact(0).normal;
            Vector3 planeBottom = transform.up;

            if (Vector3.Dot(contactNormal, planeBottom) < 0.0f) // If the bottom is not sufficiently aligned
            {
                AddReward(-1000f);
                EndEpisode();
                Debug.Log("Upside Down on runway");
            }
        }
    }
}

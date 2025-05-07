using JetBrains.Annotations;
using System.Threading;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.Experimental.GlobalIllumination;

public class PlaneAgent : Agent
{
    [Header("Speed")]
    public float thrustPower = 15f;
    public float pitchSpeed = 0.25f;
    public float yawSpeed = 0.25f;
    public float rollSpeed = 0.25f;
   
    [Header("Raycast")]
    public float raycastDistance = 25f;
    public LayerMask obstacleLayer;

    [Header("Other")]
    public GameObject[] checkpoints;
    [SerializeField] private int currentCheckpoint = 0;
    [SerializeField] private int checkpointCounter;
    [SerializeField] private GameObject checkpoint;

    private bool updraft;
    private bool takingOff;
    private float timeSinceTakeoff;
    private float timeSinceLastCheckpoint = 0;

    private Rigidbody rb;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        currentCheckpoint = Random.Range(0, checkpoints.Length -1);
        checkpoint = checkpoints[currentCheckpoint];
        transform.position = new Vector3(Random.Range(60, 125), 54, Random.Range(205,240));
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        transform.rotation = Quaternion.Euler(0, 90, 0);
        checkpointCounter = 0;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        AddRaycastObservations(sensor, Vector3.forward); 
        AddRaycastObservations(sensor, Quaternion.Euler(0, 45, 0) * Vector3.forward);
        AddRaycastObservations(sensor, Quaternion.Euler(0, -45, 0) * Vector3.forward);

        AddRaycastObservations(sensor, Quaternion.Euler(45, -45, 0) * Vector3.forward);
        AddRaycastObservations(sensor, Quaternion.Euler(45, 45, 0) * Vector3.forward);
        AddRaycastObservations(sensor, Quaternion.Euler(45, -45, 0) * Vector3.forward);

        AddRaycastObservations(sensor, Vector3.back);
        AddRaycastObservations(sensor, Vector3.right);
        AddRaycastObservations(sensor, Vector3.left);
        AddRaycastObservations(sensor, Vector3.down);

        sensor.AddObservation(transform.rotation);
        sensor.AddObservation(transform.forward);
        sensor.AddObservation(transform.up);

        sensor.AddObservation(transform.InverseTransformPoint(checkpoint.transform.position));
        sensor.AddObservation(Vector3.Distance(transform.position, checkpoint.transform.position));
        sensor.AddObservation(Vector3.Dot(transform.forward, (checkpoint.transform.position - transform.position).normalized));

        sensor.AddObservation(rb.linearVelocity);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float pitch = actions.ContinuousActions[0];
        float roll = actions.ContinuousActions[1];
        float yaw = actions.ContinuousActions[2];
        float thrust = actions.ContinuousActions[3];
        // may need to be reverted to 0
        if(thrust < 0)
        {
            thrust = 0;
        }
        updraft = false;
        if (thrust > 0.5)
        {
            updraft = true;
        }
        rb.AddRelativeTorque(Vector3.right * pitch * pitchSpeed);
        rb.AddRelativeTorque(Vector3.back * roll * rollSpeed);
        rb.AddRelativeTorque(Vector3.up * yaw * yawSpeed);
        rb.AddRelativeForce(Vector3.forward * thrust * thrustPower);
    }

    private void FixedUpdate()
    {
        if(transform.position.x > 500 || transform.position.x < 0)
        {
            AddReward(-100);
            Debug.Log("Too far Away");
            EndEpisode();
        }
        if (transform.position.z > 500 || transform.position.z < 0)
        {
            AddReward(-100);
            Debug.Log("Too far Away");
            EndEpisode();
        }
        if (transform.position.y > 250 || transform.position.y < 50)
        {
            AddReward(-100);
            Debug.Log("Too high Up");
            EndEpisode();
        }

        
        float rollAngle = Vector3.Angle(transform.up, Vector3.up);
        if (rollAngle > 90)
        {
            AddReward(-100);
            Debug.Log("Upside Down");
            EndEpisode();
        }
        if (rollAngle < 15 && transform.position.y > 60)
        {
            AddReward(0.001f);
        }
        else if (rollAngle > 15 && transform.position.y > 60)
        {
            AddReward(-0.0005f * rollAngle );
        }

        Vector3 directionToCheckpoint = (checkpoint.transform.position - transform.position).normalized;
        float forwardAlignment = Vector3.Dot(transform.forward.normalized, directionToCheckpoint);
        if (forwardAlignment >= 0.85f && transform.position.y > 60)
        {
            AddReward(0.001f);
        }
        else if (forwardAlignment < 0.85f && transform.position.y > 60)
        {
            AddReward(-0.001f * (1.0f - forwardAlignment));
        }
        

        float distanceToCheckpoint = Vector3.Distance(transform.position, checkpoint.transform.position);
        AddReward(25f / distanceToCheckpoint);

        timeSinceLastCheckpoint += Time.deltaTime;
        AddReward(-0.00002f * timeSinceLastCheckpoint);

        if (takingOff)
        {
            transform.position += (Vector3.up * 2.5f * Time.deltaTime);
            timeSinceTakeoff += Time.deltaTime;
            AddReward(-0.1f);
        }
        if (timeSinceTakeoff > 3f)
        {
            takingOff = false;
        }

        if (updraft)
        {
            rb.AddForce(Vector3.up * 7f);
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Ground"))
        {
            AddReward(-100);
            Debug.Log("Touched Ground");
            EndEpisode();
        }
        else if (collision.gameObject.CompareTag("Plane"))
        {
            AddReward(-100f);
            Debug.Log("Touched Plane");
            EndEpisode();
        }
        else if (collision.gameObject.CompareTag("Runway"))
        {
            takingOff = true;
        }
    }

    private void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.CompareTag("Runway"))
        {
            timeSinceTakeoff = 0;
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("Checkpoint"))
        {
            if(other.gameObject == checkpoint)
            {
                int nextCheckpoint = Random.Range(0, checkpoints.Length);
                while (nextCheckpoint == currentCheckpoint)
                {
                    nextCheckpoint = Random.Range(0, checkpoints.Length);
                }
                currentCheckpoint = nextCheckpoint;
                checkpoint = checkpoints[currentCheckpoint];
                AddReward(500.0f);
                checkpointCounter++;
                timeSinceLastCheckpoint = 0;
                Debug.Log("Collected Checkpoint number " + checkpointCounter);
            }
        }
    }

    private void AddRaycastObservations(VectorSensor sensor, Vector3 direction){
        if (Physics.Raycast(transform.position, transform.TransformDirection(direction), out RaycastHit hit, raycastDistance, obstacleLayer, QueryTriggerInteraction.Ignore))
        {
            //AddReward(0.00001f);
            sensor.AddObservation(hit.distance / raycastDistance);
        }
        else
        {
            // No obstacle detected
            sensor.AddObservation(1.0f);
        }
    }

    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, transform.position + transform.TransformDirection(Vector3.forward) * raycastDistance);

        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position, transform.position + transform.TransformDirection(Quaternion.Euler(0, 45, 0) * Vector3.forward) * raycastDistance);

        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position, transform.position + transform.TransformDirection(Quaternion.Euler(0, -45, 0) * Vector3.forward) * raycastDistance);

        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, transform.position + transform.TransformDirection((Quaternion.Euler(45, 0, 0) * Vector3.forward)) * raycastDistance);

        Gizmos.color = Color.blue;
        Gizmos.DrawLine(transform.position, transform.position + transform.TransformDirection(Quaternion.Euler(45, 45, 0) * Vector3.forward) * raycastDistance);

        Gizmos.color = Color.green;
        Gizmos.DrawLine(transform.position, transform.position + transform.TransformDirection(Quaternion.Euler(45, -45, 0) * Vector3.forward) * raycastDistance);

        Gizmos.color = Color.cyan;
        Gizmos.DrawLine(transform.position, transform.position + transform.TransformDirection(Vector3.back) * raycastDistance);
        Gizmos.DrawLine(transform.position, transform.position + transform.TransformDirection(Vector3.right) * raycastDistance);
        Gizmos.DrawLine(transform.position, transform.position + transform.TransformDirection(Vector3.left) * raycastDistance);
        Gizmos.DrawLine(transform.position, transform.position + transform.TransformDirection(Vector3.down) * raycastDistance);

        Color lineColor = Color.magenta;

        if (gameObject != null && checkpoints[currentCheckpoint] != null)
        {
            Debug.DrawLine(gameObject.transform.position, checkpoints[currentCheckpoint].transform.position, lineColor);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manuelle Steuerung für Tests
        var continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxis("Vertical");
        continuousActions[1] = Input.GetKey(KeyCode.Q) ? 1f : Input.GetKey(KeyCode.E) ? -1f : 0f;
        continuousActions[2] = Input.GetAxis("Horizontal");
        continuousActions[3] = Input.GetKey(KeyCode.Space) ? 1f : 0f;
    }
}

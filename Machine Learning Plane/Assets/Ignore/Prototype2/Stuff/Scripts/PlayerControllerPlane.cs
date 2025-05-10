using UnityEngine;

public class PlayerControllerPlane : MonoBehaviour
{
    public float thrustPower = 15f;
    public float pitchSpeed = 2f;
    public float yawSpeed = 2f;
    public float rollSpeed = 2f;

    private bool takingOff;
    private float timeSinceTakeoff;
    private Rigidbody rb;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        
        float pitch = Input.GetAxis("Vertical");
        float yaw = Input.GetAxis("Horizontal");
        float thrust = Input.GetKey(KeyCode.Space) ? 1f : 0f;
        float roll = 0;
        if (Input.GetKey(KeyCode.Q)){
            roll = -1;
        }
        if (Input.GetKey(KeyCode.E)) { 
            roll = 1;
        }

        rb.AddRelativeTorque(Vector3.right * pitch * pitchSpeed);   // Pitch
        rb.AddRelativeTorque(Vector3.back * roll * rollSpeed);       // Roll
        rb.AddRelativeTorque(transform.up.normalized * yaw * yawSpeed);    // Yaw
        rb.AddRelativeForce(Vector3.forward * thrust * thrustPower);    // Thrust

        if (thrust > 0.25f)
        {
            rb.AddForce(Vector3.up * 2.5f);
        }
        if (takingOff) {
            transform.position += (Vector3.up * 1.75f * Time.deltaTime);
            timeSinceTakeoff += Time.deltaTime;
        }
        if(timeSinceTakeoff > 3f)
        {
            takingOff = false;
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Runway"))
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
}

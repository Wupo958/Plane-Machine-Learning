using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class ChangePlanesScript : MonoBehaviour
{
    public GameObject[] cameras;

    public TextMeshProUGUI textUI;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        switchCamera(0);
        switchText("Plane 1, 500k steps, 6 min");
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            switchCamera(0);
            switchText("Plane 1, 500k steps, 6 min");
        }
        if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            switchCamera(1);
            switchText("Plane 2, 2.5mil steps, 32 min");
        }
        if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            switchCamera(2);
            switchText("Plane 3, 10mil steps, 2 hours");
        }
        if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            switchCamera(3);
            switchText("Plane 4, 25mil steps, 5 hours");
        }
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            Application.Quit();
        }
    }

    private void switchCamera(int j){
        for (int i = 0;  i < cameras.Length; i++){
            if( i != j ){
                cameras[i].SetActive(false);
            }else{
                cameras[i].SetActive(true);
            }
        }
    }

    private void switchText(string text){
        textUI.text = "Active Plane: " + text;
    }
}

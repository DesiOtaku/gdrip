This is a program is to aid dentists in diagnosis of various oral pathology that may be evident in a patient's radiograph. It is right now pre-alpha so use it only if you know what you are doing!

Carries detection todo List:
- Try to find the background via the "seed growing" method
- See if I can use the sobel operator
- Try finding the boundry of the enamel for the first tooth and then follow it
- Use http://free.pages.at/easyfilter/bresenham.html for the Bézier curve
- Edge detection via "chunks" in differences

General function todo List:
- Sharpen
- Undo/redo stack
- Reduce noise
- Edge enhance
- Be able to "mark" locations like "this place is where you have the carries"
- Have a "reference" pic for patients to see exactly what is wrong
- Look for a way to extract an image from a DICOM file with having to implement all of it
- Redo the ImageProcessor to use proper lines and markings

from typing import List
from college import Student, Course
import utils


def calculate_gpa(student: Student, courses: List[Course]) -> float:
    '''
    This function takes a student and a list of course
    It should compute the GPA for the student
    The GPA is the sum(hours of course * grade in course) / sum(hours of course)
    The grades come in the form: 'A+', 'A' and so on.
    But you can convert the grades to points using a static method in the course class
    To know how to use the Student and Course classes, see the file "college.py"  
    '''
    course_hours = 0.0
    course_grades = 0.0
    sum_hours = 0.0
    sum_hrspts = 0.0
    gpa = 0.0
    for i in range(len(courses)):
        course_hours = courses[i].hours
        course_grades = courses[i].convert_grade_to_points(
            courses[i].grades[student.id])
        sum_hours += course_hours
        sum_hrspts += (course_hours*course_grades)
        gpa = sum_hrspts/sum_hours
    return gpa

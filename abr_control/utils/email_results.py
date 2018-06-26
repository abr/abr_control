from abr_control.utils.paths import cache_dir
import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import encoders
import traceback

def send_email(from_email=None, from_password=None, to_email=None,
        subject=None, body=None, file_location=None):
    if file_location is None:
        print("No attachment")
    if from_email is None:
        print("ERROR: Must provide source email to send from")
    if to_email is None:
        print("ERROR: Must provide email to send to")
    if from_password is None:
        print("ERROR: Must provide password for source email address")
    if subject is None:
        subject = "abr_control:results"
    if body is None:
        body = "This is an automated message using the abr_control repo"

    msg = MIMEMultipart()

    msg['From'] = from_email
    msg['To'] = to_email
    msg['Subject'] = subject

    body = body

    msg.attach(MIMEText(body, 'plain'))

    part = MIMEBase('application', 'octet-stream')

    if file_location is not None:
        filename = file_location
        attachment = open(filename, "rb")
        part.set_payload((attachment).read())
        encoders.encode_base64(part)
        part.add_header('Content-Disposition', "attachment; filename= %s" % filename)
        msg.attach(part)

    try:
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.starttls()
        server.login(from_email, from_password)
        text = msg.as_string()
        server.sendmail(from_email, to_email, text)
        server.quit()
    except:
        print('Error: unable to send email')
        print(traceback.format_exc())

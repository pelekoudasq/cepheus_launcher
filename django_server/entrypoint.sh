#!/bin/bash
echo "!!!!!!!!!!!!!!!!!!!!>>>>>>>>>>>>>>>>>>>>>"
# python manage.py makemigrations --noinput
# python manage.py migrate --noinput
python3 manage.py runserver 0.0.0.0:8000
echo "!!!!!!!!!!!!!!!!!!!!>>>>>>>>>>>>>>>>>>>>>"

from dashboard.views import home_view
from django.urls import path

urlpatterns = [
    path('', home_view, name='users-home'),
]
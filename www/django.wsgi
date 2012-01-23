import os, sys
sys.path.append('/home/iain/www/mte482/git/www')
sys.path.append('/home/iain/www/mte482/git/www/mte481')
os.environ['DJANGO_SETTINGS_MODULE'] = 'mte481.settings'

import django.core.handlers.wsgi

application = django.core.handlers.wsgi.WSGIHandler()

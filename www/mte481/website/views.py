# Create your views here.
from django.http import HttpResponse, Http404
from django.shortcuts import render_to_response

# A list of available pages.  
# The key 'page' specifies the url the page serves under, and also identifies 
# the page template.
# The title is used in the html <title>, and also for navigation.
page_list = [
  {'name': 'about', 'title': 'Overview'},
  {'name': 'progress', 'title':'Progress'},
  {'name': 'design', 'title': 'Design Details'},
  {'name': 'group', 'title': 'Team Members'},
]

def root(request):
  """ Requests for the root are just redirected to the about page """
  resp = HttpResponse()
  resp.status_code = 301
  resp['Location'] = "http://"+request.get_host()+"/about"
  return resp

def page(request, page):
  """ Handles requests for the various pages given in page_list """

  if not page in [p['name'] for p in page_list]:
    raise Http404

  # Find the title that goes with the page 
  title = None
  for p in page_list:
    if p['name'] == page:
      title = p['title']

  return render_to_response(page+'.html', 
      {'name': page, 'title': title, 'page_list': page_list})


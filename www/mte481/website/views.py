# Create your views here.
from django.http import HttpResponse, Http404
from django.shortcuts import render_to_response

def root(request):
  """ Requests for the root are just redirected to the about page """
  resp = HttpResponse()
  resp.status_code = 301
  resp['Location'] = "http://"+request.get_host()+"/about"
  return resp

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

def page(request, page):
  """ Handles requests for the various pages given in page_list """
  print("page: "+page)

  # Find the title that goes with the page 
  title = None
  for p in page_list:
    if p['name'] == page:
      title = p['title']

  if title is None:
    raise Http404

  return render_to_response(page+'.html', 
      {'name': page, 'title': title, 'page_list': page_list})

# A list of available stylesheet templates
style_list = [
  "main",
]

# Parameters for stylesheets to use:
style_args = {
  'palette': {
    'bg': '#6e84d6',
    'bg_head': '#4867d6',
    'bg_head_content': '#1435ad',
    'bg_text': '#fff',
    'bg_nav': '#2c3e82',
    'text': '#000',
    'text_head': '#fff',
    'text_nav': '#fff',
  },
  'page_width': '70%',
  'head_height': '105px',
}

def style(request, style):
  """ Handles requests for stylesheet templates """
  print("style: "+style)

  # Ensure the requested style exists:
  if not style in style_list:
    raise Http404

  response = render_to_response(style+'.css', style_args);
  response['Content-Type'] = 'text/css; charset=utf-8'
  return response


import React from 'react';
import { Redirect } from '@docusaurus/router';

export default function Login() {
  // Redirect to the new sign-in page
  return <Redirect to="/auth/signin" />;
}